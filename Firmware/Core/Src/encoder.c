/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Gestione encoder con Timer in Input Capture (TIM3, TIM4)
  *          e Timer in Encoder mode (TIM1, TIM2).
  ******************************************************************************
  */

#include "encoder.h"      // include del tuo header con EncoderData_t
#include "stm32f4xx_hal.h"// per TIM_HandleTypeDef, HAL_TIM_xxx
#include <math.h>         // per fabsf, se non già incluso

/* Dichiarazioni extern dei timer generati da CubeMX */
extern TIM_HandleTypeDef htim1; // Encoder mode per Encoder 1
extern TIM_HandleTypeDef htim2; // Encoder mode per Encoder 2
extern TIM_HandleTypeDef htim3; // Input Capture per Encoder 1
extern TIM_HandleTypeDef htim4; // Input Capture per Encoder 2

/* =============================================================================
 *  Costanti e macro (se non presenti altrove)
 * =============================================================================
 */
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

#define VELOCITY_THRESHOLD  0.01f
#define MAX_UINT16          65535
#define TIMER_FREQ          84000000.0f   // esempio 84 MHz su STM32F4

/* =============================================================================
 *  Variabili Globali (se le dichiari in 'globals.c', togli la definizione qui!)
 * =============================================================================
 *
 * Se già definite in un altro file .c, qui andranno come "extern".
 *
 * Di seguito un esempio se vuoi allocarle qui. Altrimenti, rimuovi e usa extern.
 */
/*
EncoderData_t g_Encoder1;
EncoderData_t g_Encoder2;
*/

/* =============================================================================
 *  Implementazione delle funzioni
 * =============================================================================
 */
void ENC_Init(void)
{
    /* ---------------- Encoder 1 ---------------- */
    // Inizializza tutti i campi
    g_Encoder1 = (EncoderData_t){
        .cpr                = 400,
        .rpm_max            = 3000,
        .velocity           = 0.0f,
        .position           = 0,
        .position_radians   = 0.0f,
        .capture_old        = 0,
        .impulse_time       = 0,
        .direction_capture  = 0,
        .new_data           = false,

        .timer_to_seconds   = 1.0f / TIMER_FREQ,
        .count_to_rad       = (2.0f * M_PI) / 400.0f,
        .ticks_to_rpm       = (60.0f * TIMER_FREQ) / 400.0f,
        .velocity_limit     = 3000.0f * 2.0f
    };

    /* ---------------- Encoder 2 ---------------- */
    g_Encoder2 = (EncoderData_t){
        .cpr                = 400,
        .rpm_max            = 3000,
        .velocity           = 0.0f,
        .position           = 0,
        .position_radians   = 0.0f,
        .capture_old        = 0,
        .impulse_time       = 0,
        .direction_capture  = 0,
        .new_data           = false,

        .timer_to_seconds   = 1.0f / TIMER_FREQ,
        .count_to_rad       = (2.0f * M_PI) / 400.0f,
        .ticks_to_rpm       = (60.0f * TIMER_FREQ) / 400.0f,
        .velocity_limit     = 3000.0f * 2.0f
    };

    /*
     * Se vuoi abilitare un prescaler calcolato:
     * (Attento: se commentato, i timer funzionano col prescaler impostato da CubeMX).
     */
    /*
    float min_period_usec_1 = (60.0f * 1e6f) / (g_Encoder1.rpm_max * g_Encoder1.cpr);
    uint32_t optimal_prescaler_1 = (uint32_t)((TIMER_FREQ * min_period_usec_1) / 1e6f);
    if (optimal_prescaler_1 < 1) {
        optimal_prescaler_1 = 1;
    }
    htim3.Init.Prescaler = optimal_prescaler_1 - 1;

    float min_period_usec_2 = (60.0f * 1e6f) / (g_Encoder2.rpm_max * g_Encoder2.cpr);
    uint32_t optimal_prescaler_2 = (uint32_t)((TIMER_FREQ * min_period_usec_2) / 1e6f);
    if (optimal_prescaler_2 < 1) {
        optimal_prescaler_2 = 1;
    }
    htim4.Init.Prescaler = optimal_prescaler_2 - 1;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK || HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        // Gestione errore (possibile assert)
    }
    */

    // Avvia TIM3 e TIM4 in modalità Input Capture con interrupt
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

    // Avvia TIM1 e TIM2 in modalità Encoder
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

/**
  * @brief  ISR di input capture: qui calcoliamo la durata dell'impulso
  *         (tempo tra due fronti consecutivi).
  * @param  htim: puntatore a TIM_HandleTypeDef che ha generato l'interrupt
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // Encoder 1
    {
        // Legge il valore attuale di cattura
        uint32_t capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        // Ora capture_val rappresenta già il tempo passato dall’ultimo fronte,
        // purché azzeriamo il contatore subito dopo aver letto.
        g_Encoder1.impulse_time = capture_val;

        // Direzione dal timer in encoder mode (TIM1 in hardware encoder)
        g_Encoder1.direction_capture = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) ? -1 : 1;

        // Segnaliamo che ci sono nuovi dati
        g_Encoder1.new_data = true;

        // Azzeriamo il contatore di TIM3, così riparte da 0 per il prossimo fronte
        __HAL_TIM_SET_COUNTER(htim, 0);

        // Aggiorna anche la posizione software leggendola dal TIM1
        int16_t count_now = __HAL_TIM_GET_COUNTER(&htim1);
        if (count_now != 0)
        {
            int16_t direction_hw = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) ? -1 : 1;
            g_Encoder1.position += direction_hw * count_now;
            __HAL_TIM_SET_COUNTER(&htim1, 0);
        }
    }
    else if (htim->Instance == TIM4) // Encoder 2
    {
        uint32_t capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        g_Encoder2.impulse_time = capture_val;
        g_Encoder2.direction_capture = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ? -1 : 1;
        g_Encoder2.new_data = true;

        __HAL_TIM_SET_COUNTER(htim, 0);  // azzera il contatore di TIM4

        int16_t count_now = __HAL_TIM_GET_COUNTER(&htim2);
        if (count_now != 0)
        {
            int16_t direction_hw = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ? -1 : 1;
            g_Encoder2.position += direction_hw * count_now;
            __HAL_TIM_SET_COUNTER(&htim2, 0);
        }
    }
}


/**
  * @brief  Funzione di aggiornamento periodico (ad es. chiamata ogni ms).
  *         Converte impulse_time (tick) in velocità (RPM), esegue filtri e
  *         calcola la posizione in radianti.
  */
void ENC_Update(void)
{
    /* ------------------- Encoder 1 ------------------- */
    if (g_Encoder1.new_data)
    {
        g_Encoder1.new_data = false;

        // L'impulso è direttamente il conteggio letto dall'Input Capture
        float dt = (float)g_Encoder1.impulse_time;
        if (dt > 0.0f)
        {
            // velocity = direction * (ticks_to_rpm / dt)
            float new_velocity = g_Encoder1.direction_capture * (g_Encoder1.ticks_to_rpm / dt);

            // Limite massimo di velocità
            if (fabsf(new_velocity) > g_Encoder1.velocity_limit)
            {
                new_velocity = 0.0f;
            }
            g_Encoder1.velocity = new_velocity;
        }
        else
        {
            // Se dt fosse 0 (o molto piccolo), evitiamo divisione
            g_Encoder1.velocity = 0.0f;
        }
    }

    // Filtro soglia minima
    if (fabsf(g_Encoder1.velocity) < VELOCITY_THRESHOLD)
    {
        g_Encoder1.velocity = 0.0f;
    }

    // Calcolo della posizione in radianti
    g_Encoder1.position_radians = g_Encoder1.position * g_Encoder1.count_to_rad;


    /* ------------------- Encoder 2 ------------------- */
    if (g_Encoder2.new_data)
    {
        g_Encoder2.new_data = false;

        float dt = (float)g_Encoder2.impulse_time;
        if (dt > 0.0f)
        {
            float new_velocity = g_Encoder2.direction_capture * (g_Encoder2.ticks_to_rpm / dt);

            if (fabsf(new_velocity) > g_Encoder2.velocity_limit)
            {
                new_velocity = 0.0f;
            }
            g_Encoder2.velocity = new_velocity;
        }
        else
        {
            g_Encoder2.velocity = 0.0f;
        }
    }

    if (fabsf(g_Encoder2.velocity) < VELOCITY_THRESHOLD)
    {
        g_Encoder2.velocity = 0.0f;
    }

    g_Encoder2.position_radians = g_Encoder2.position * g_Encoder2.count_to_rad;
}

