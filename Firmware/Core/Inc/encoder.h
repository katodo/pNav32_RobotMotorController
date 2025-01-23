#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>   // Necessario per il tipo bool
#include <math.h>      // Per M_PI (se disponibile in toolchain)

/* Se M_PI non fosse definito dalla toolchain, puoi forzarlo:
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif
*/

/* Costanti principali */
#define VELOCITY_THRESHOLD  0.01f           // Soglia minima per velocità
#define MAX_UINT16          65535           // Timer 16 bit
#define TIMER_FREQ          84000000.0f     // Frequenza base del timer (es: 84 MHz su STM32F4)

/**
 * Struttura dati per un singolo encoder.
 */
typedef struct
{
    /* Parametri iniziali */
    uint32_t cpr;
    float    rpm_max;

    /* Variabili di lavoro */
    float    velocity;         // Velocità in RPM
    int32_t  position;         // Posizione in "tick"
    float    position_radians; // Posizione in radianti

    /* Per gestire Input Capture */
    uint32_t capture_old;      // Valore "old" del timer (lettura precedente)
    uint32_t impulse_time;     // Durata dell'impulso in tick
    int16_t  direction_capture;
    bool     new_data;

    /* Precalcoli */
    float timer_to_seconds;
    float count_to_rad;
    float ticks_to_rpm;
    float velocity_limit;

} EncoderData_t;


/* Variabili globali (dichiarate in encoder.c con extern qui) */
extern volatile EncoderData_t g_Encoder1;
extern volatile EncoderData_t g_Encoder2;


/* Prototipi delle funzioni */
void ENC_Init(void);
void ENC_Update(void);

/*
 * La callback di Input Capture è già definita dal framework HAL con la firma:
 *   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
 * Quindi non serve ridefinirla qui se non come "extern", se preferisci:
 */
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
