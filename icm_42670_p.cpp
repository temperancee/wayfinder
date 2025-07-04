#include "icm_42670_p.h"
#include "Arduino.h"
#include "HardwareSerial.h"
#include "freertos/idf_additions.h"
#include <cstdint>
#include <sys/types.h>

/* 
 * Binary semaphore for alerting the data_read function that the fifo_ready_ISR
 * has run. We should use task notifications for this, but I can't figure out a
 * good way to pass the data_ready TaskHandle from the .ino file to the ISR
 */
static SemaphoreHandle_t data_read_sem;


const SPISettings spi_settings(24000000, MSBFIRST, SPI_MODE0);


/***********************************************************************************************************************************************/
/* ISRs */

static void data_ready_ISR( void )
{
    // This flag compares the priority of the task woken by the semaphore and the task
    // that was running before the ISR and runs the higher priority one accordingly
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Serial.println("misery");

    // flag fifo read task to resume
    xSemaphoreGiveFromISR(data_read_sem, &higher_priority_task_woken);

    // Run the higher priority task
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

/***********************************************************************************************************************************************/
/* Functions */

uint8_t reg_write(icm_42670_p *icm, uint8_t reg_addr, uint8_t len, const uint8_t *buf)
{
    SPI.beginTransaction(spi_settings);
    digitalWrite(icm->CS_PIN, LOW); // Pull CS low to begin transaction
    // The device expects a R/W bit, then a 7 bit addr. We pass the function a 7 bit addr
    // and W=0, so by default this is already configured properly!
    SPI.transfer(reg_addr);
    for (int i=0; i<len; i++) {
        SPI.transfer(buf[i]);
    }
    digitalWrite(icm->CS_PIN, HIGH); // write CS high to end transaction
    SPI.endTransaction();
    return 0;
}


uint8_t reg_read(icm_42670_p *icm, uint8_t reg_addr, uint8_t len, uint8_t *buf)
{
    SPI.beginTransaction(spi_settings);
    digitalWrite(icm->CS_PIN, LOW); // Pull CS low to begin transaction
    // The device expects a R/W bit, then a 7 bit addr. We pass the function a 7 bit addr
    // then edit the MSB bit to be R=1
    SPI.transfer(reg_addr | (1 << 7));
    for (int i=0; i<len; i++) {
        buf[i] = SPI.transfer(0x00); // send a value of 0 to read the byte returned
    }
    digitalWrite(icm->CS_PIN, HIGH); // write CS high to end transaction
    SPI.endTransaction();
    return 0;
}

uint8_t icm_initialise(icm_42670_p *icm, const uint8_t INT_PIN, const uint8_t CS_PIN)
{
    // Attach pins to ICM object
    icm->CS_PIN = CS_PIN;
    icm->INT_PIN = INT_PIN;

    // Set up CS pins as outputs - Arduino does not handle CS pins automatically
    pinMode(CS_PIN, OUTPUT);
    SPI.begin();

    /* Initialise icm mutex */
    icm->mutex = xSemaphoreCreateMutex();
    /* Create binary semaphore for fifo_ready_ISR - NOTE: I feel like either this should be part of the icm object, or the icm mutex should be a global like this */
    data_read_sem = xSemaphoreCreateBinary();

    // TODO: Change to data read reg config 
    reg_write(icm, INT_CONFIG, 1, &VAL_INT_CONFIG); // Configure INT1
    reg_write(icm, PWR_MGMT0, 1, &VAL_PWR_MGMT0); // Turn on the accel and gyro
    vTaskDelay(300 / portTICK_PERIOD_MS); // The datasheet says not to write to any register for 200us after turning the gyro and accel on
    reg_write(icm, GYRO_CONFIG0, 1, &VAL_GYRO_CONFIG0); // Configure Gyro FS and ODR
    reg_write(icm, ACCEL_CONFIG0, 1, &VAL_ACCEL_CONFIG0); // Configure Accel FS and ODR
    // TODO: Low pass filter goes here
    reg_write(icm, INT_SOURCE0, 1, &VAL_INT_SOURCE0); // Route data ready interrupt to INT1

    /* Interrupt */
    pinMode(INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), data_ready_ISR, FALLING);

    /* Check if we can read whoami register. If not, return error */
    uint8_t who_am_i;
    reg_read(icm, WHO_AM_I, 1, &who_am_i);
    if (who_am_i != 0x67) {
        return -1;
    }

    return 0;
}

/***********************************************************************************************************************************************/
/* Tasks */

void data_read_task(void *arg)
{
    icm_42670_p *icm = (icm_42670_p *)arg;
    uint8_t data_buf[GYRO_ACCEL_REG_SIZE];
    uint8_t buf[1];

    while(1) {

        // Serial.println("CS PIN :");
        // Serial.println(icm->CS_PIN);
        /* Wait until fifo_ready_ISR tells us there is data in the FIFO*/
        if (xSemaphoreTake(data_read_sem, portMAX_DELAY) == pdTRUE) {


            reg_read(icm, ACCEL_DATA_X1, GYRO_ACCEL_REG_SIZE, data_buf);
            
            /* accelerometer data is 16 bit 2s complement, stored in 2 separate 8-bit registers */
            int16_t accel_raw[3];
            accel_raw[0] = ((int16_t)data_buf[0] << 8) | ((int16_t)data_buf[1]);
            accel_raw[1] = ((int16_t)data_buf[2] << 8) | ((int16_t)data_buf[3]);
            accel_raw[2] = ((int16_t)data_buf[4] << 8) | ((int16_t)data_buf[5]);

            /* gyro data is 16 bit 2s complement, stored in 2 separate 8-bit registers */
            int16_t gyro_raw[3];
            gyro_raw[0] = ((int16_t)data_buf[6] << 8) | ((int16_t)data_buf[7]);
            gyro_raw[1] = ((int16_t)data_buf[8] << 8) | ((int16_t)data_buf[9]);
            gyro_raw[2] = ((int16_t)data_buf[10] << 8) | ((int16_t)data_buf[11]);

            float accel_x = (accel_raw[0] / (float)ACCEL_SENS) * 9.81;
            float accel_y = (accel_raw[1] / (float)ACCEL_SENS) * 9.81;
            float accel_z = (accel_raw[2] / (float)ACCEL_SENS) * 9.81;

            float gyro_x = (float)gyro_raw[0] / GYRO_SENS; // - gyro_roll_offset;
            float gyro_y = (float)gyro_raw[1] / GYRO_SENS; // - gyro_pitch_offset;
            float gyro_z = (float)gyro_raw[2] / GYRO_SENS; // - gyro_yaw_offset;

            /* Critical section - use mutex to protect mpu6050 object */
            if (xSemaphoreTake(icm->mutex, portMAX_DELAY) == pdTRUE) {
                icm->accel[0] = accel_x;
                icm->accel[1] = accel_y;
                icm->accel[2] = accel_z;

                icm->gyro[0] = gyro_x;
                icm->gyro[1] = gyro_y;
                icm->gyro[2] = gyro_z;

                xSemaphoreGive(icm->mutex);
            }

        }

    }
}



