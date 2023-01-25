extern	volatile uint16_t adc_values[];
extern	volatile	uint16_t adc_values[5];
extern	uint32_t 	bufferAdc1Dma[];
extern	uint32_t 	bufferAdc3Dma[];
extern	volatile 	uint32_t 	adc_val;
extern 	ADC_HandleTypeDef hadc1;
extern 	DMA_HandleTypeDef hdma_adc1;
extern uint32_t  dmaTransferComplete;
extern uint32_t  dmaHalfTransferComplete;
