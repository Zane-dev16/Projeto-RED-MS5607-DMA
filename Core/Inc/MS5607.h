/*
 * BARO1.h
 *
 *  Created on: 17 Jun 2020
 *      Author: linus
 */


#ifndef MS5607_H_
#define MS5607_H_

#include "main.h"
#include "i2c.h"

#define BARO1_INIT() \
  { \
    .addr = 0x76 << 1, \
	.i2c_bus = &hi2c1, \
	.delay = 100, \
  }

#define BARO2_INIT() \
  { \
    .addr = 0x77 << 1, \
	.i2c_bus = &hi2c2, \
	.delay = 100, \
  }

enum ms5607_stage {
  MS_TEMPERATURE_REQ = 0,
  MS_PRESSURE_REQ = 1,
};

// *** structs *** //

typedef struct ms5607_dev {

	// Hardware Configuration
	uint8_t addr;
	I2C_HandleTypeDef* i2c_bus;
	uint8_t delay;
	uint16_t cal[6];
	uint32_t D1;
	uint32_t D2;
} MS5607;

extern uint8_t ms5607_init(struct ms5607_dev * dev);
extern void ms5607_prep_pressure(struct ms5607_dev * dev, uint8_t * dat);
extern void ms5607_read_pressure(struct ms5607_dev * dev, uint8_t * dat);
extern void ms5607_convert(struct ms5607_dev * dev, float * p, float * t);


#endif /* MS5607_H_ */
