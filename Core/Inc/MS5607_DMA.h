#ifndef MS5607_DMA_H
#define MS5607_DMA_H

#include <stdint.h>
#include "main.h"
#include "i2c.h"
#include "MS5607.h"

extern void ms5607_dma_prep_pressure(struct ms5607_dev * dev);
extern void ms5607_dma_prep_temp();
extern void ms5607_dma_request_data();
extern void ms5607_dma_read_pressure(struct ms5607_dev * dev);
extern void ms5607_dma_read_temp(struct ms5607_dev * dev);
extern void ms5607_convert(struct ms5607_dev * dev, float * p, float * t);
extern void ms5607_dev_init(struct ms5607_dev * dev);

#endif // MS5607_H
