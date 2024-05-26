#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "main.h"
#include "i2c.h"
#include <math.h>

#define LSM6DSO32_ADDR 0x6A << 1 // Assuming SDO/SA0 is connected to GND
#define CTRL2_G        0x11 // Register address of CTRL2_G
#define CTRL3_C 0x12
#define OUTX_L_G 0x22

HAL_StatusTypeDef LSM6DSO32_Init(void);
HAL_StatusTypeDef LSM6DSO32_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
float lsm6dso32_from_fs500_to_mdps(int16_t lsb);

#endif // LSM6DSO32_H
