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
const uint8_t MTR1_PIN = D5;
const uint8_t MTR2_PIN = D1;
const uint8_t MTR3_PIN = D4;
const uint8_t MTR4_PIN = D2;

/* Constants */
// PID
#define PID_t 1.0f          // PID loop time step
#define outer_P_roll 1
#define outer_I_roll 1
#define outer_D_roll 1
#define outer_P_pitch 1
#define outer_I_pitch 1
#define outer_D_pitch 1

#define inner_P_roll 1
#define inner_I_roll 1
#define inner_D_roll 1
#define inner_P_pitch 1
#define inner_I_pitch 1
#define inner_D_pitch 1
#define inner_P_yaw 1
#define inner_I_yaw 1
#define inner_D_yaw 1

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

// Roll and pitch state - used to store aspects in the PID loop.
typedef struct state_tag {
    float roll;
    float pitch;
} state;

// Roll, pitch, and yaw state - used in the inner PID loop 

typedef struct full_state_tag {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} full_state;

/* Declare global objects */
icm_42670_p icm;
kalman kf;
state desired_state;
full_state desired_rates;

// Task handles for timer callbacks
static TaskHandle_t predict_handle = NULL;
static TaskHandle_t measurement_handle = NULL;
static TaskHandle_t pid_handle = NULL;
static TimerHandle_t predict_timer = NULL;
static TimerHandle_t measurement_timer = NULL;
static TimerHandle_t pid_timer = NULL;

/***************************
 *        FUNCTIONS        *
 ***************************/

float sqr(float x)
{
    return x*x;
}

/**
 * @brief   Calculates the output of the PID equations 
 *
 * @param[in]   err   The error term
 * @param[in]   prev_err   The error term of the previous iteration
 * @param[in]   p_term     Stores the proportional term of the PID sum
 * @param[in]   i_term     Stores the integral term of the PID sum
 * @param[in]   d_term     Stores the derivative term of the PID sum
 * @param[in]   P          The proportional parameter (K_P)
 * @param[in]   I          The integral parameter (K_I)
 * @param[in]   D          The derivative parameter (K_D)
 *
 * @return   The output of the calculation 
 */
float pid_calc(float err, float prev_err, float p_term, float i_term, float d_term, float P, float I, float D)
{
    p_term = P*err;
    i_term += I*(err + prev_err*PID_t)/2;
    d_term = D*(err - prev_err)/PID_t;

    return p_term + i_term + d_term;
}

/**
 * @brief   Converts the PID rate output to a PWM frequency for a motor
 *
 * @param[in]   rate    The PID rate
 */
void rate_to_motor_pwm(float rate)
{
  map(rate, 0, 90, 0, 4000);
}


/**
 * @brief   Sets up motor PWM resolutions for finer control
 */
void motor_setup()
{
  // Increase resolution to 12 bits (0 - 4096) for finer control
  analogWriteResolution(MTR1_PIN, 12);
  analogWriteResolution(MTR2_PIN, 12);
  analogWriteResolution(MTR3_PIN, 12);
  analogWriteResolution(MTR4_PIN, 12);
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
    } else if (timer_handle == measurement_timer) {
        vTaskNotifyGiveFromISR(measurement_handle, &higher_priority_task_woken);
    } else {
        vTaskNotifyGiveFromISR(pid_handle, &higher_priority_task_woken);
    }
    

    // Run the higher priority task
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

/***************************
 *          TASKS          *
 ***************************/

/* PID Control Loops */
void pid_task(void *arg)
{
  state outer_err;
  state prev_outer_err;
  state outer_p_term;
  state outer_i_term;
  state outer_d_term;
  state pid_rates;

  full_state inner_err;
  full_state prev_inner_err;
  full_state inner_p_term;
  full_state inner_i_term;
  full_state inner_d_term;
  full_state pid_motor_inputs;

  while (1) {
  
    /* Outer PID loop - calculates the desired angular roll/pitch rates */
    // Update error terms  
    outer_err.roll = desired_state.roll - kf.roll;
    outer_err.pitch = desired_state.pitch - kf.pitch;

    // Calculate PID terms
    pid_rates.roll = pid_calc(outer_err.roll, prev_outer_err.roll, outer_p_term.roll, outer_i_term.roll, outer_d_term.roll, outer_P_roll, outer_I_roll, outer_D_roll);
    pid_rates.pitch = pid_calc(outer_err.pitch, prev_outer_err.pitch, outer_p_term.pitch, outer_i_term.pitch, outer_d_term.pitch, outer_P_pitch, outer_I_pitch, outer_D_pitch);

    // Store error terms for next iteration
    prev_outer_err.roll = outer_err.roll;
    prev_outer_err.pitch = outer_err.pitch;

    /* Inner PID loop - calculates the motor inputs based on desired rates. Note that yaw is just the raw desired rate, it doesn't come from the previous PID loop */
    // Update error terms
    inner_err.roll = pid_rates.roll - icm.gyro[0]; 
    inner_err.pitch = pid_rates.pitch - icm.gyro[1]; 
    inner_err.pitch = desired_rates.yaw - icm.gyro[2]; 

    // Calculate the PID terms
    pid_motor_inputs.roll = pid_calc(inner_err.roll, prev_inner_err.roll, inner_p_term.roll, inner_i_term.roll, inner_d_term.roll, inner_P_roll, inner_I_roll, inner_D_roll);
    pid_motor_inputs.pitch = pid_calc(inner_err.pitch, prev_inner_err.pitch, inner_p_term.pitch, inner_i_term.pitch, inner_d_term.pitch, inner_P_pitch, inner_I_pitch, inner_D_pitch);
    pid_motor_inputs.yaw = pid_calc(inner_err.yaw, prev_inner_err.yaw, inner_p_term.yaw, inner_i_term.yaw, inner_d_term.yaw, inner_P_yaw, inner_I_yaw, inner_D_yaw);
    
    // Store error terms for next iteration
    prev_inner_err.roll = inner_err.roll;
    prev_inner_err.pitch = inner_err.pitch;
    prev_inner_err.yaw = inner_err.yaw;
    
    // Motor matrix
    // TODO: Make this a real function
    mtr_update([
               desired_rates.throttle - pid_motor_inputs.roll - pid_motor_inputs.pitch - pid_motor_inputs.yaw,
               desired_rates.throttle - pid_motor_inputs.roll + pid_motor_inputs.pitch + pid_motor_inputs.yaw,
               desired_rates.throttle + pid_motor_inputs.roll + pid_motor_inputs.pitch - pid_motor_inputs.yaw,
               desired_rates.throttle + pid_motor_inputs.roll - pid_motor_inputs.pitch + pid_motor_inputs.yaw
              ]);
  } 
}

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
    pid_timer = xTimerCreate("PID Timer", PID_t / portTICK_PERIOD_MS, pdTRUE, (void *)2, task_timer_callback);

    // Start tasks
    xTaskCreatePinnedToCore(predict_task, "predict_task", 2048, NULL, 2, &predict_handle, app_cpu);
    xTaskCreatePinnedToCore(measurement_task, "measurement_task", 2048, NULL, 2, &measurement_handle, app_cpu);
    xTaskCreatePinnedToCore(pid_task, "pid_task", 2048, NULL, 3, &pid_handle, app_cpu);
    xTaskCreatePinnedToCore(print_task, "print_task", 2048, NULL, 1, NULL, app_cpu);

    // Start timer
    configASSERT(predict_timer != NULL);
    xTimerStart(predict_timer, portMAX_DELAY);
    configASSERT(measurement_timer != NULL);
    xTimerStart(measurement_timer, portMAX_DELAY);
    configASSERT(pid_timer != NULL);
    xTimerStart(pid_timer, portMAX_DELAY);

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}

void loop() 
{
    // Execution should never reach here
}
