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

	buf[0] = 0x44;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	// need to wait 3 ms
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

void ms5607_convert(struct ms5607_dev * dev, float * p, float * t)
{
	//calculate calibration values
	uint16_t c1 = dev->cal[0];
	uint16_t c2 = dev->cal[1];
	uint16_t c3 = dev->cal[2];
	uint16_t c4 = dev->cal[3];
	uint16_t c5 = dev->cal[4];
	uint16_t c6 = dev->cal[5];

	uint32_t D1 = dev->D1;
	uint32_t D2 = dev->D2;

	//calculations from datasheet
	float dt = (float)D2 - c5 * 256.0;
	float OFF = c2 * pow(2.0, 17) + (c4 * dt)/64.0;
	float SENS = c1 * pow(2.0, 16) + (c3 * dt)/128.0;
	float TEMP = 2000.0 + dt * c6/(pow(2.0, 23));
	float pressure = ((float)D1 * SENS/(pow(2.0, 21)) - OFF)/(pow(2.0, 15));

	float T2 = 0., OFF2 = 0., SENS2 = 0.;
	if(TEMP < 2000)
	{
	  T2 = dt * dt / pow(2.0,31);
	  OFF2 = 61.0 * (TEMP - 2000.0) * (TEMP - 2000.0)/pow(2.0,4);
	  SENS2 = 2.0 * (TEMP - 2000.0) * (TEMP - 2000.0);
	  if(TEMP < -1500)
	  {
	    OFF2 += 15.0 * (TEMP + 1500)*(TEMP + 1500.0);
	    SENS2 += 8.0 * (TEMP + 1500)*(TEMP + 1500.0);
	  }
	}

	TEMP-=T2;
	OFF-=OFF2;
	SENS-=SENS2;
	TEMP/=100;
	pressure=(((float)(D1*SENS)/pow(2,21)-OFF)/pow(2,15));

	*t = TEMP;
	*p = pressure;

	//printf("MS pressure is %4.2f Pa\n", pressure);
	//printf("MS temp is %4.2f deg\n", TEMP);

}


