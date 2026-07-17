// Use only core 1 because I don't know how to use multicores yet
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#include "icm_42670_p.h"
#include <math.h>


/***************************
 * DEFINITIONS AND GLOBALS *
 ***************************/

/* Define pins */
const uint8_t INT_PIN = D0;
const uint8_t CS_PIN = D6;

/* Constants */

// Kalman Filter
#define t 2.0f              // Kalman filter predict time step in ms
#define MEASURE_TIME 10.0f      // Kalman filter measurement time step in ms
#define SIGMA_INIT 0.01f    // Initial values on diagonal of sigma (i.e. initial roll and pitch variances)
#define R_INIT 0.001f       // Initial values on diagonal of R (process noise)
#define Q_INIT 0.011f       // Initial values on diagonal of Q (measurement noise)

#define RAD_TO_DEG 57.2957795131f

/* Typedefs */
// Kalman Filter state and variance - initial state estimate is that roll and pitch are 0, as the drone will be lay flat.
typedef struct KalmanFilter {
    float roll;
    float pitch;
    float Sigma[4]; // Store as 4-dim array rather than 2x2 nested array for simplicity - 0 = 11, 1 = 12, 2 = 21, 3 = 22
    float R[2];     // R and Q are always diagonal matrices, so only store diagonal values - 0 = 11, 1 = 22
    float Q[2];
    SemaphoreHandle_t mutex;
} kalman;

/* Declare global objects */
icm_42670_p icm;
kalman kf;

// Task handles for timer callbacks
static TaskHandle_t predict_handle = NULL;
static TaskHandle_t measurement_handle = NULL;
static TimerHandle_t predict_timer = NULL;
static TimerHandle_t measurement_timer = NULL;

/***************************
 *        FUNCTIONS        *
 ***************************/

float sqr(float x)
{
    return x*x;
}

/***************************
 *        Callbacks        *
 ***************************/

void task_timer_callback(TimerHandle_t timer_handle)
{
    // This flag compares the priority of the task woken by the notification and the task
    // that was running before the ISR and runs the higher priority one accordingly
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Check predict handle is not NULL - it shouldn't be, as we assign it in setup()
    configASSERT( predict_handle != NULL );

    // Send task notification to run predict/measurement step of Kalman Filter
    if (timer_handle == predict_timer) {
        vTaskNotifyGiveFromISR(predict_handle, &higher_priority_task_woken);
    } else {
        vTaskNotifyGiveFromISR(measurement_handle, &higher_priority_task_woken);
    } 

    // Run the higher priority task
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

/***************************
 *          TASKS          *
 ***************************/


/* Kalman Filter prediction step */
void predict_task(void *arg)
{

    while (1) {

        // Run on timer interrupt - notification acts as binary semaphore
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 1) {

            // Read the data
            read_sensor_data(&icm);

            // Protect kf and icm objects with mutex
            if ( (xSemaphoreTake(kf.mutex, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(icm.mutex, portMAX_DELAY) == pdTRUE) ) {
                // Predict step
                kf.roll += (t/1000.0)*icm.gyro[0];
                kf.pitch += (t/1000.0)*icm.gyro[1];

                kf.Sigma[0] += kf.R[0];  
                kf.Sigma[3] += kf.R[1];

                xSemaphoreGive(kf.mutex);
                xSemaphoreGive(icm.mutex);
            }
            
        }
    
    }
}

/* Kalman Filter measurement step*/
void measurement_task(void *arg)
{
    // Roll and pitch angles as measured by the accelerometer
    float m_roll;
    float m_pitch;
    // Kalman gain
    float k_gain[4]; // Stored as a 1D array, just like Sigma
    float k_det;    // Determinant used in Kalman Gain calculation

    while (1) {

        // Run on timer interrupt - notification acts as binary semaphore
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 1) {

            // Read the data
            read_sensor_data(&icm);

            // Protect icm object with mutex
            if (xSemaphoreTake(icm.mutex, portMAX_DELAY) == pdTRUE) {
                // Initially, icm.accel values are 0. If this runs before the first data_ready interrupt comes in, m_roll and pitch will be NaN due to div by 0
                if (icm.accel[0] == 0) {
                    m_roll = 0;
                    m_pitch = 0;
                } else {
                    m_roll = atan( icm.accel[1] / sqrt( sqr(icm.accel[0]) + sqr(icm.accel[2]) ))*RAD_TO_DEG;
                    m_pitch = atan( -icm.accel[0] / sqrt( sqr(icm.accel[1]) + sqr(icm.accel[2]) ))*RAD_TO_DEG;
                }
                
                xSemaphoreGive(icm.mutex);
            }
            // Protect kf object with mutex
            if (xSemaphoreTake(kf.mutex, portMAX_DELAY) == pdTRUE) {

                // Kalman Gain
                k_det = 1 / ( (kf.Sigma[0]+kf.Q[0])*(kf.Sigma[3]+kf.Q[1]) - kf.Sigma[1]*kf.Sigma[2] );
                k_gain[0] = k_det * ( kf.Sigma[0]*(kf.Sigma[3] + kf.Q[1]) - kf.Sigma[1]*kf.Sigma[2] );
                k_gain[1] = k_det * ( kf.Sigma[1]*kf.Q[0] );
                k_gain[2] = k_det * ( kf.Sigma[2]*kf.Q[1] );
                k_gain[3] = k_det * ( kf.Sigma[3]*(kf.Sigma[0] + kf.Q[0]) - kf.Sigma[1]*kf.Sigma[2] );


                // State vector
                kf.roll += k_gain[0]*(m_roll - kf.roll) + k_gain[1]*(m_pitch - kf.pitch);
                kf.pitch += k_gain[2]*(m_roll - kf.roll) + k_gain[3]*(m_pitch - kf.pitch);

                // State variance
                kf.Sigma[0] = kf.Sigma[0]*(1 - k_gain[0]) + k_gain[1]*kf.Sigma[2];
                kf.Sigma[1] = kf.Sigma[1]*(1 - k_gain[0]) + k_gain[1]*kf.Sigma[3];
                kf.Sigma[2] = kf.Sigma[2]*(1 - k_gain[3]) + k_gain[2]*kf.Sigma[0];
                kf.Sigma[3] = kf.Sigma[3]*(1 - k_gain[3]) + k_gain[2]*kf.Sigma[1];

                xSemaphoreGive(kf.mutex);
            }

        }
        
    }
}

/* Prints to serial port */
void print_task(void *arg)
{

    uint8_t buf[1];

    while (1) {
        // Lock with mutex
        if (xSemaphoreTake(kf.mutex, portMAX_DELAY) == pdTRUE) { 
            Serial.print("Roll = ");
            Serial.print(kf.roll);
            Serial.print(" deg, ");
            Serial.print("Pitch = ");
            Serial.print(kf.pitch);
            Serial.println(" deg");
            Serial.println("---------------------------");

            xSemaphoreGive(kf.mutex);

            vTaskDelay(200 / portTICK_PERIOD_MS);
        }       
    }
}



/***************************
 *          SETUP          *
 ***************************/

void setup() 
{
    Serial.begin(115200);
    // Provide time for Serial to start up
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println();
    Serial.println("--- Begin ---");

    // Initialise ICM
    if (icm_initialise(&icm, CS_PIN) == -1) {
        Serial.println("ICM initialisation failed!!");
    }
      
    // Initialise KF 
    kf.mutex = xSemaphoreCreateMutex();
    kf.Sigma[0] = kf.Sigma[3] = SIGMA_INIT;
    kf.Sigma[1] = kf.Sigma[2] = 0;
    kf.R[0] = kf.R[1] = R_INIT;
    kf.Q[0] = kf.Q[1] = Q_INIT;

    // Create timer
    predict_timer = xTimerCreate("Predict Timer", t / portTICK_PERIOD_MS, pdTRUE, (void *)0, task_timer_callback);
    measurement_timer = xTimerCreate("Measurement Timer", MEASURE_TIME / portTICK_PERIOD_MS, pdTRUE, (void *)1, task_timer_callback);

    // Start tasks
    xTaskCreatePinnedToCore(predict_task, "predict_task", 2048, NULL, 2, &predict_handle, app_cpu);
    xTaskCreatePinnedToCore(measurement_task, "measurement_task", 2048, NULL, 2, &measurement_handle, app_cpu);
    xTaskCreatePinnedToCore(print_task, "print_task", 2048, NULL, 1, NULL, app_cpu);

    // Start timer
    configASSERT(predict_timer != NULL);
    xTimerStart(predict_timer, portMAX_DELAY);
    configASSERT(measurement_timer != NULL);
    xTimerStart(measurement_timer, portMAX_DELAY);

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}

void loop() 
{
    // Execution should never reach here
}
