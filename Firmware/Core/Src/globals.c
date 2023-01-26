#include "main.h"
#include "adc.h"
#include <stdbool.h>

volatile uint32_t 	bufferAdc1Dma[6];
volatile uint32_t 	bufferAdc3Dma[2];

uint32_t  dmaTransferComplete;
uint32_t  dmaHalfTransferComplete;
