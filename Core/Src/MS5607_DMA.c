#include <MS5607_DMA.h>

#include <stdio.h>
#include <math.h>

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;


#define MS5607_ADDR 0x76 << 1
volatile uint8_t conversion_complete = 0;
volatile uint8_t dma_rx_complete = 0;
volatile uint8_t dma_tx_complete = 0;

void ms5607_dev_init(struct ms5607_dev * dev) {
    dev->addr = MS5607_ADDR; // Set default sensor address
    dev->i2c_bus = &hi2c1;
    dev->delay = 100; // Set default delay value (in milliseconds)
}

void ms5607_dma_wait()
{
	while (HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
    while (HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
}

void ms5607_dma_prep_pressure(struct ms5607_dev * dev) {
    uint8_t buf[1] = {0x44};
    HAL_StatusTypeDef status;
    status=HAL_I2C_Master_Transmit_DMA(dev->i2c_bus, MS5607_ADDR, buf, 1);
    while (!dma_tx_complete);
    dma_tx_complete = 0;
    HAL_Delay(2);
    if (status != HAL_OK) {
        // Handle the error (e.g., log the error, reset the peripheral, etc.)
        printf("HAL_I2C_Master_Transmit_DMA failed with status %d\n", status);
        return;
    }
}

void ms5607_dma_prep_temp()
{
    uint8_t buf[1] = {0x54};
    HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5607_ADDR, buf, 1);
    ms5607_dma_wait(); // Wait for DMA transfer to complete
}

void ms5607_dma_request_data()
{
    uint8_t buf[1] = {0x00};  // Command to read ADC result
    HAL_StatusTypeDef status;
    status=HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5607_ADDR, buf, 1);
    ms5607_dma_wait(); //
    if (status != HAL_OK) {
        // Handle the error (e.g., log the error, reset the peripheral, etc.)
        printf("HAL_I2C_Master_Transmit_DMA failed with status %d\n", status);
        return;
    }
    // Note: This function doesn't receive data, so there's no DMA receive operation
}

void ms5607_dma_read_pressure(struct ms5607_dev * dev)
{
    uint8_t buf[3];
    ms5607_dma_wait();
    HAL_I2C_Master_Receive_DMA(&hi2c1, MS5607_ADDR, buf, 3);
    ms5607_dma_wait(); // Wait for DMA transfer to complete
    dev->D1 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | (uint32_t)buf[2];
}

void ms5607_dma_read_temp(struct ms5607_dev * dev)
{
    uint8_t buf[3];
    HAL_I2C_Master_Receive_DMA(&hi2c1, MS5607_ADDR, buf, 3);
    ms5607_dma_wait(); // Wait for DMA transfer to complete
    dev->D2 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | (uint32_t)buf[2];
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

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	hi2c->State = HAL_I2C_STATE_READY;
        dma_rx_complete = 1;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	hi2c->State = HAL_I2C_STATE_READY;
        dma_tx_complete = 1;
    }
}
