#include "main.h"
#include "adc.h"
#include <stdbool.h>

volatile	uint16_t adc_values[5];


uint32_t 	bufferAdc1Dma[6];
uint32_t 	bufferAdc3Dma[6];

volatile 	uint32_t 	adc_val;
extern 		ADC_HandleTypeDef hadc1;
extern 		DMA_HandleTypeDef hdma_adc1;
uint32_t  dmaTransferComplete;
uint32_t  dmaHalfTransferComplete;
