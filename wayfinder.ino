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
// Kalman Filter
#define t 2.0f              // Kalman filter predict time step in ms
#define MEASURE_TIME 1.0f      // Kalman filter measurement time step in ms
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
} kalman_t;

/* Declare global objects */
icm_42670_p icm;
kalman_t kf;


/***************************
 *        FUNCTIONS        *
 ***************************/

float sqr(float x)
{
    return x*x;
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
            // Protect kf and icm objects with mutex
            if ( (xSemaphoreTake(kf.mutex, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(icm.mutex, portMAX_DELAY) == pdTRUE) ) {
                // Predict step
                kf.roll += (t/1000.0)*icm.gyro[0];
                kf.pitch += (t/1000.0)*icm.gyro[1];

                kf.Sigma[0] += kf.R[0];   // See formulae in obsidian notes
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

            // Protect icm object with mutex
            if (xSemaphoreTake(icm.mutex, portMAX_DELAY) == pdTRUE) {
                // Initially, icm.accel values are 0. If this runs before the first data_ready interrupt comes in, m_roll and pitch will be NaN due to div by 0
                if (icm.accel[0] == 0) {
                    m_roll = 0;
                    m_pitch = 0;
                } else {
                    m_roll = atan( icm.accel[0] / sqrt( sqr(icm.accel[1]) + sqr(icm.accel[2]) ))*RAD_TO_DEG;
                    m_pitch = atan( icm.accel[1] / sqrt( sqr(icm.accel[0]) + sqr(icm.accel[2]) ))*RAD_TO_DEG;
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
        if (xSemaphoreTake(icm.mutex, portMAX_DELAY) == pdTRUE) { 
            Serial.print("Accel X = ");
            Serial.print(icm.accel[0]);
            Serial.print(" m/s^2, ");
            Serial.print("Accel Y = ");
            Serial.print(icm.accel[1]);
            Serial.print(" m/s^2, ");
            Serial.print("Accel Z = ");
            Serial.print(icm.accel[2]);
            Serial.println(" m/s^2");
            Serial.println("---------------------------");

            /* 
             * Read the data ready status register to unlatch the interrupt 
             * I actually have it set to pulse, but it latches anyway, for some reason
             */
            reg_read(&icm, 0x39, 1, buf);
            xSemaphoreGive(icm.mutex);

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

    if (icm_initialise(&icm, INT_PIN, CS_PIN) == -1) {
        Serial.println("ICM initialisation failed!!");
   }
    
    /* Pass ICM object to FIFO read task so it can be updated with new measurements */
    xTaskCreatePinnedToCore(data_read_task, "data_read_task", 2048, (void *)&icm, 2, NULL, app_cpu);
    xTaskCreatePinnedToCore(print_task, "print_task", 2048, NULL, 1, NULL, app_cpu);

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}

void loop() 
{
    // Execution should never reach here
}
