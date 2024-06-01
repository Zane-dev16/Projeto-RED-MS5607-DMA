/*
 * ms5607.c
 *
 *  Created on: 29 Aug 2020
 *      Author: linus
 */


/*
 *
 * Usage:
 *
 * MS5607 BARO1 = BARO1_INIT();
 *
 * ms5607_init(BARO1);
 *
 * ms5607_prep_temp(BARO1);
 *
 * delay 3ms
 *
 * ms5607_prep_pressure(BARO1, raw_data);
 *
 * delay 3ms
 *
 * ms5607_read_pressure(BARO1, raw_data);
 *
 * ms5607_convert(BARO1, p, t);
 *
 * raw_data = LSB_T, CSB_T, MSB_T, LSB_P, CSB_P, MSB_P
 *
 */


#include <MS5607.h>
#include <stdio.h>
#include <math.h>


uint8_t ms5607_init(struct ms5607_dev * dev)
{
	if (HAL_I2C_GetState(dev->i2c_bus) != HAL_I2C_STATE_READY)
	{
		printf("i2c not ready!\n");
	} else {
		printf("i2c is ready!\n");
	}
	HAL_StatusTypeDef _ret;
	_ret = HAL_I2C_IsDeviceReady(dev->i2c_bus, dev->addr, 10, dev->addr);
	if ( _ret != HAL_OK )
	{
		printf("BARO setup fail\n");
		printf("Errorcode: %d\n", _ret);
		return 0;
	}

	//get factory calibration data
	//reset (advised in datasheet)

	uint8_t reset_code[1];
	reset_code[0] = 0x1E;
	_ret = HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, reset_code, 1, dev->delay);

	HAL_Delay(100);

	//6 calibration values with each having 2 bytes

	//get each calibration value (c1 - c6 in datasheet)
	uint8_t get_add;
	uint8_t buf[2];

	for(int i = 1; i < 7; i++){

		//standard commands (see datasheet)
		get_add = 0b10100000;
		get_add = get_add + 2*i;

		_ret = HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, &get_add, 1, dev->delay);
		HAL_Delay(15);
		_ret = HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 2, dev->delay);
		dev->cal[i-1] = (uint16_t)(buf[0] << 8) | buf[1];

		if ( _ret != HAL_OK )
			{
				printf("MS5607 cal read fail\n");
			}
	}


	printf("BARO setup success\n");
	return 1;
}

void ms5607_prep_pressure(struct ms5607_dev * dev, uint8_t * dat)
{
	uint8_t buf[3];
	buf[0] = 0x00;

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 3, dev->delay);

	dev->D1 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | (uint32_t)buf[2];
	dat[0] = buf[0];
	dat[1] = buf[1];
	dat[2] = buf[2];

	buf[0] = 0x54;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	// need to wait 3 ms
}

void ms5607_read_pressure(struct ms5607_dev * dev, uint8_t * dat)
{
	uint8_t buf[3];
	buf[0] = 0x00;

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 3, dev->delay);

	dev->D2 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | (uint32_t)buf[2];
	dat[0] = buf[0];
	dat[1] = buf[1];
	dat[2] = buf[2];

	buf[0] = 0x44;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	// need to wait 3 ms
}


