/* A basic driver for the ICM-42670-P using Arduino's SPI library
 * Fetches gyroscope and accelerometer data
 * This driver is not suited to be configured - it hardcodes many configuration choices
 * Made for Seeeduino XIAO ESP32S3
 */

#ifndef ICM42670P_H
#define ICM42670P_H

#include <Arduino.h>
#include <SPI.h>

/*
- DEVICE_CONFIG - I'm not changing SPI mode from 1, so don't need this.
- (?) DRIVE_CONFIG3 - what the fuck is slew rate
- INT_CONFIG
- ACCEL_DATA_xx
- GYRO_DATA_xx
- PWR_MGMT0
- GYRO_CONFIG0
- ACCEL_CONFIG0
- GYRO_CONFIG1
- ACCEL_CONFIG1
- (note)APEX_CONFIG1 - We don't need this for basics, but pay attention to the description of the DMP_ODR bits, as we may have to change ACCEL_ODR (in ACCEL_CONFIG1) bits accordingly (may not matter if we don't use APEX?)
- FIFO_CONFIG1
- FIFO_CONFIG2
- FIFO_CONFIG3
- INT_SOURCE0 - configuring what interrupts go to INT1
- INTF_CONFIG0 - big endian format is probably good, but check
- INT_STATUS_DRDY - data ready int register
- INT_STATUS
- FIFO_COUNTH/L
- FIFO_DATA
- WHO_AM_I
*/


#define WHO_AM_I 0x75
#define INT_CONFIG 0x06 

#define ACCEL_DATA_X1 0x0B
#define ACCEL_DATA_X0 0x0C
#define ACCEL_DATA_Y1 0x0D
#define ACCEL_DATA_Y0 0x0E
#define ACCEL_DATA_Z1 0x0F
#define ACCEL_DATA_Z0 0x10

#define GYRO_DATA_X1 0x11
#define GYRO_DATA_X0 0x12
#define GYRO_DATA_Y1 0x13
#define GYRO_DATA_Y0 0x14
#define GYRO_DATA_Z1 0x15
#define GYRO_DATA_Z0 0x16

#define PWR_MGMT0 0x1F
#define GYRO_CONFIG0 0x20
#define GYRO_CONFIG1 0x23
#define ACCEL_CONFIG0 0x21
#define ACCEL_CONFIG1 0x24
#define INT_SOURCE0 0x2B
#define INT_STATUS 0x3A

/* 
 * Below are fixed configuration values written to registers, with explanations for what they do
 * A more robust driver would allow these to be configured by the user 
 */

const uint8_t VAL_PWR_MGMT0 = 0b00001111; // Turn accel and gyro on in low noise mode

/*
 * |-------------------------------------------------------|
 * | VAL  | Full Scale Range (+- deg/s) | Sens (LSB/deg/s) |
 * | ---- | --------------------------- | ---------------- |
 * | 0x00 | 250                         | 131              |
 * | 0x08 | 500                         | 65.5             |
 * | 0x10 | 1000                        | 32.8             |
 * | 0x18 | 2000                        | 16.4             |
 * |-------------------------------------------------------|

 * |-----------------------------------------------|
 * | VAL  | Full Scale Range (+- g) | Sens (LSB/g) |
 * | ---- | ----------------------- | ------------ |
 * | 0x00 | 2                       | 16384        |
 * | 0x08 | 4                       | 8192         |
 * | 0x10 | 8                       | 4096         |
 * | 0x18 | 16                      | 2048         |
 * |-----------------------------------------------|
 */
const uint8_t VAL_GYRO_CONFIG0 = 0b00100101; // Gyro FS=+-1000dps, ODR=1600Hz (fastest)
const uint8_t VAL_ACCEL_CONFIG0 = 0b00100101; // Accel FS=+-8g, ODR = 1600Hz (fastest)
// Define ACCEL_SENS and GYRO_SENS while we're at it - in good code these would be edited along with the above 2 lines in a user facing function. This is not a good driver.
#define ACCEL_SENS 4096
#define GYRO_SENS 32.8
const uint8_t VAL_GYRO_CONFIG1 = 0b00000000; // TODO: LOW PASS FILTER SETUP
const uint8_t VAL_ACCEL_CONFIG1 = 0b00000000; // TODO: LOW PASS FILTER SETUP. Also overaging set to default, because we aren't using low power mode

#define GYRO_ACCEL_REG_SIZE 12 // Size of gyro and accel registers, used for reading them in the data_read_task

/*
 * Structs
 */

typedef struct icm_42670_p {
    float gyro[3];
    float accel[3];
    SemaphoreHandle_t mutex;
    uint8_t CS_PIN;
    uint8_t INT_PIN;
} icm_42670_p;


/*
 * Functions
 */


/**
 * @brief   Reads/writes n bytes starting from the specified register
 * 
 * @param[in]       reg_addr     Register to begin read/write at
 * @param[in]       len          Number of registers to read from sequentially
 * @param[in,out]   buf          Either used to store the read values, or filled by the user to contain values to be written
 * @return  0 to indicate success. Might add error types later
 */
uint8_t reg_read(icm_42670_p *icm, uint8_t reg_addr, uint8_t len, uint8_t *buf);
uint8_t reg_write(icm_42670_p *icm, uint8_t reg_addr, uint8_t len, uint8_t *buf);

/**
 * @brief   Performs basic set up for the ICM-42670-P, and initialises the object
 *
 * @param[in,out]   *icm        Pointer to the ICM object to initialise
 * @param[in]       CS_PIN      Chip Select pin
 * @return  0 to indicate success, -1 for error.
 */
uint8_t icm_initialise(icm_42670_p *icm, const uint8_t CS_PIN);

 /**
  * @brief   Reads data from the gyroscope and accelerometer, and updates the global ICM object 
  *
  * @param[in,out]   *icm        Pointer to the ICM object to initialise
  */
void read_sensor_data(icm_42670_p *icm);



#endif // #ifndef ICM42670P_H
