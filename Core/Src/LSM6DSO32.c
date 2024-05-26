#include "LSM6DSO32.h"

HAL_StatusTypeDef LSM6DSO32_Init(void) {
    // Configure CTRL2_G register
    uint8_t ctrl2_g_data = 0x00; // Default configuration
    HAL_StatusTypeDef ret;


    // Set output data rate (ODR) and full-scale range (FS) for gyroscope
    // Example: Set ODR to 833 Hz and FS to ±125 dps
    ctrl2_g_data |= (0x07 << 4) | (0x01 << 1);
    ret = HAL_I2C_Mem_Write(&hi2c1, LSM6DSO32_ADDR, CTRL2_G, I2C_MEMADD_SIZE_8BIT, &ctrl2_g_data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
    	return ret;
    }
    HAL_Delay(100);
    // Configure CTRL3_C register (enable automatic register address incrementation)
    uint8_t ctrl3_c_data = 0x00;
    ctrl3_c_data |= (1 << 2);

    // Write to CTRL3_C register
    ret = HAL_I2C_Mem_Write(&hi2c1, LSM6DSO32_ADDR, CTRL3_C, I2C_MEMADD_SIZE_8BIT, &ctrl3_c_data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
    	return ret;
    }
    HAL_Delay(100);
    return HAL_OK;
}


HAL_StatusTypeDef LSM6DSO32_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t gyro_data[6];
    HAL_StatusTypeDef ret;

    // Read 6 bytes from the OUTX_L_G register (auto-increment enabled)
    ret = HAL_I2C_Mem_Read(&hi2c1, LSM6DSO32_ADDR, OUTX_L_G, I2C_MEMADD_SIZE_8BIT, gyro_data, 6, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
    	return ret;
    }
    HAL_Delay(100);
    // Combine high and low bytes
    *gyro_x = (int16_t)(gyro_data[1] << 8 | gyro_data[0]);
    *gyro_y = (int16_t)(gyro_data[3] << 8 | gyro_data[2]);
    *gyro_z = (int16_t)(gyro_data[5] << 8 | gyro_data[4]);
    return HAL_OK;
}

float lsm6dso32_from_fs500_to_mdps(int16_t lsb) {
    return ((float)lsb) * 0.004375;
}
