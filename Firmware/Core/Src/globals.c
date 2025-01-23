#include "globals.h"
#include "encoder.h"

/* Variabili globali DMA */
volatile uint32_t bufferAdc1Dma[6];
volatile uint32_t bufferAdc3Dma[2];

uint32_t dmaTransferComplete = 0;
uint32_t dmaHalfTransferComplete = 0;

/* Definizioni delle variabili encoder */
volatile EncoderData_t g_Encoder1;
volatile EncoderData_t g_Encoder2;

