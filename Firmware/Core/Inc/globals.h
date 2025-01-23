#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include "encoder.h"

/* Variabili globali DMA */
extern volatile uint32_t bufferAdc1Dma[];
extern volatile uint32_t bufferAdc3Dma[];

extern uint32_t dmaTransferComplete;
extern uint32_t dmaHalfTransferComplete;

/* Variabili globali encoder */
extern volatile EncoderData_t g_Encoder1;
extern volatile EncoderData_t g_Encoder2;

#endif /* GLOBALS_H */


