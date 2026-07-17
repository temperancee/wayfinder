# Wayfinder

Wayfinder is a custom PCB I designed, containing an ICM-42670-P inertia measurement unit (IMU), a XIAO ESP32S3 Sense, and a barometer. The software is written in C using Arudino libraries and the Espressif-IDF version of FreeRTOS. The ESP32 communicates via I2C with the IMU via a custom driver I wrote. The accelerometer and gyroscope readings are then passed to a Kalman filter, which fuses the sensor data into roll and pitch angles, so you know the orientation of the board!

## Details

The Kalman filter is split into a predict task and a measurement task. The predict task reads gyroscope data, and incorporates it into the angle values using Euler integration. The measurement task reads accelerometer data, maps it to angles, and then incorporates it in the measurement step of the Kalman filter

The predict task runs every two milliseconds, and the measurement task runs every ten. There isn't really any reason for this in this project, although, for higher-end IMUs, accelerometer data is often generated slower than gyroscope data, so such a timing setup would be sensible.

Despite being called `icm_42670_p.cpp`, the code is all C. Arduino requires that external (non `.ino`) files are `.cpp`, rather than `.c`.

The IMU driver is exceedingly simple, and makes no use of the IMU's FIFO buffer. It simply provides commands for reading from the registers that store the gyroscope and accelerometer readings.

<!-- Originally intended to be a quadcopter flight computer, Wayfinder settled for the modest life of being an IMU board. Built using a custom PCB with a XIAO ESP32S3 Sense daughter board, and an ICM-42670-P IMU, this board outputs -->

## Running

If you want to use the driver, simply copy `icm_42670_p.h` and `icm_42670_p.cpp` into your Arduino project. 

## Licesnse

No licesnse. Do as you will with the code.
