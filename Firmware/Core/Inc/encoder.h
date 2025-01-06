#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief Struttura che contiene i dati di posizione e velocità per ogni encoder.
 */
typedef struct
{
    /* Dati da TIM1/TIM2 (encoder quadratura) */
    int32_t position;      /**< Posizione a 32 bit (conteggio esteso). */
    int32_t velocity;      /**< Velocità calcolata in ENC_Update (ticks/ms). */

    uint16_t lastCount;    /**< Ultimo valore CNT a 16 bit per gestione overflow. */
    int32_t  lastPosition; /**< Usato per il calcolo di velocity a 1ms. */

    /* Dati da TIM3/TIM4 in input capture */
    uint32_t icLastCapture;  /**< Ultima lettura CCR (cattura). */
    float    icVelocityTPS;  /**< Velocità in ticks/s (calcolata con input capture). */

} EncoderData_t;

typedef enum
{
    ENCODER_1 = 0,
    ENCODER_2
} EncoderId_t;

/* Variabili globali (istanze per i 2 encoder) */
extern EncoderData_t g_Encoder1;
extern EncoderData_t g_Encoder2;

/* --- TIM1/TIM2 in quadratura --- */
void ENC_Init(void);
void ENC_Update(void);  /* Da richiamare a 1ms per aggiornare posizione e velocity (ticks/ms). */
int32_t ENC_GetPosition(EncoderId_t id);
int32_t ENC_GetVelocity(EncoderId_t id);
void ENC_Reset(EncoderId_t id);

/* --- TIM3/TIM4 in input capture --- */
void ENC_IC_Init(void);
float ENC_GetVelocityTPS(EncoderId_t id);

/* --- Callback unica HAL per input capture (TIM3 & TIM4) --- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
