#include "encoder.h"

/* Handle generati da CubeMX (controlla i nomi nel tuo progetto) */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Strutture globali per i due encoder */
EncoderData_t g_Encoder1 = {0};
EncoderData_t g_Encoder2 = {0};

/* Prescaler e frequenze per TIM3 e TIM4 (inizialmente prescaler = 0 e clock = 84MHz).
   Se il tuo clock è differente, regola di conseguenza. */
static uint32_t s_TIM3Prescaler = 0;
static float    s_TIM3ClockFreq = 84000000.0f;  // es: 84 MHz di base (STM32F4)
static uint32_t s_TIM4Prescaler = 0;
static float    s_TIM4ClockFreq = 84000000.0f;

/* ==============================
 *  Funzioni per TIM1/TIM2
 * ============================== */
void ENC_Init(void)
{
    /* Inizializzazione di Encoder1 */
    g_Encoder1.position      = 0;
    g_Encoder1.velocity      = 0;
    g_Encoder1.lastCount     = 0;
    g_Encoder1.lastPosition  = 0;
    g_Encoder1.icLastCapture = 0;
    g_Encoder1.icVelocityTPS = 0.0f;

    /* Inizializzazione di Encoder2 */
    g_Encoder2.position      = 0;
    g_Encoder2.velocity      = 0;
    g_Encoder2.lastCount     = 0;
    g_Encoder2.lastPosition  = 0;
    g_Encoder2.icLastCapture = 0;
    g_Encoder2.icVelocityTPS = 0.0f;

    /* Avvio TIM1 e TIM2 in modalità Encoder (Quadrature) */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* Azzeriamo i contatori hardware */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void ENC_Update(void)
{
    /* === Encoder1 === */
    uint16_t currentCount1 = __HAL_TIM_GET_COUNTER(&htim1);
    int16_t delta1 = (int16_t)(currentCount1 - g_Encoder1.lastCount);
    g_Encoder1.position += delta1;
    g_Encoder1.lastCount = currentCount1;

    /* Velocità in ticks/ms */
    g_Encoder1.velocity = g_Encoder1.position - g_Encoder1.lastPosition;
    g_Encoder1.lastPosition = g_Encoder1.position;

    /* === Encoder2 === */
    uint16_t currentCount2 = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t delta2 = (int16_t)(currentCount2 - g_Encoder2.lastCount);
    g_Encoder2.position += delta2;
    g_Encoder2.lastCount = currentCount2;

    g_Encoder2.velocity = g_Encoder2.position - g_Encoder2.lastPosition;
    g_Encoder2.lastPosition = g_Encoder2.position;
}

int32_t ENC_GetPosition(EncoderId_t id)
{
    if (id == ENCODER_1) return g_Encoder1.position;
    else if (id == ENCODER_2) return g_Encoder2.position;
    return 0;
}

int32_t ENC_GetVelocity(EncoderId_t id)
{
    if (id == ENCODER_1) return g_Encoder1.velocity;
    else if (id == ENCODER_2) return g_Encoder2.velocity;
    return 0;
}

void ENC_Reset(EncoderId_t id)
{
    if (id == ENCODER_1)
    {
        g_Encoder1.position      = 0;
        g_Encoder1.velocity      = 0;
        g_Encoder1.lastPosition  = 0;
        g_Encoder1.lastCount     = 0;
        g_Encoder1.icLastCapture = 0;
        g_Encoder1.icVelocityTPS = 0.0f;
        __HAL_TIM_SET_COUNTER(&htim1, 0);
    }
    else if (id == ENCODER_2)
    {
        g_Encoder2.position      = 0;
        g_Encoder2.velocity      = 0;
        g_Encoder2.lastPosition  = 0;
        g_Encoder2.lastCount     = 0;
        g_Encoder2.icLastCapture = 0;
        g_Encoder2.icVelocityTPS = 0.0f;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
    }
}

/* ==============================
 *  Funzioni per TIM3 / TIM4 IC
 * ============================== */

/* Avvio TIM3_CH1 e TIM4_CH2 in input capture con interrupt. */
void ENC_IC_Init(void)
{
    /* Valori iniziali di prescaler */
    s_TIM3Prescaler = 0;
    s_TIM3ClockFreq = 84000000.0f;

    s_TIM4Prescaler = 0;
    s_TIM4ClockFreq = 84000000.0f;

    /* Avvio la cattura in interrupt su TIM3 e TIM4 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
}

/* Restituisce la velocità in ticks/s (calcolata via input capture) */
float ENC_GetVelocityTPS(EncoderId_t id)
{
    if (id == ENCODER_1) return g_Encoder1.icVelocityTPS;
    else if (id == ENCODER_2) return g_Encoder2.icVelocityTPS;
    return 0.0f;
}

/* ==============================
 *  Callback unica
 * ============================== */

/* Funzione di supporto: cambio dinamico del prescaler */
static void ENC_IC_AdjustPrescaler(
    TIM_HandleTypeDef *htim,
    uint32_t *pPrescaler,
    float    *pClockFreq,
    uint32_t  delta)
{
    /* Soglie di esempio: da regolare in base alla tua applicazione */
    const uint32_t DELTA_TOO_SMALL = 10;      // soglia per ridurre prescaler (velocità altissima)
    const uint32_t DELTA_TOO_LARGE = 30000;   // soglia per aumentare prescaler (velocità molto bassa)

    uint32_t newPrescaler = *pPrescaler;

    if (delta < DELTA_TOO_SMALL)
    {
        /* Velocità troppo alta -> decrementa prescaler finché > 0 */
        if (newPrescaler > 0)
        {
            newPrescaler--;
        }
    }
    else if (delta > DELTA_TOO_LARGE)
    {
        /* Velocità troppo bassa -> incrementa prescaler (se non al limite) */
        if (newPrescaler < 0xFFFF)
        {
            newPrescaler++;
        }
    }

    /* Se il prescaler è cambiato, aggiorniamo il timer */
    if (newPrescaler != *pPrescaler)
    {
        __HAL_TIM_DISABLE(htim);

        htim->Init.Prescaler = newPrescaler;
        __HAL_TIM_SET_PRESCALER(htim, newPrescaler);

        if (HAL_TIM_Base_Init(htim) != HAL_OK)
        {
            /* Gestisci errore se necessario */
        }

        /* Riavvio cattura (occhio: potremmo perdere un fronte durante la riconfigurazione) */
        if (htim->Instance == TIM3)
        {
            HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
        }
        else if (htim->Instance == TIM4)
        {
            HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
        }

        __HAL_TIM_ENABLE(htim);

        /* Nuova freq timer = 84MHz / (prescaler+1) */
        *pClockFreq = 84000000.0f / (float)(newPrescaler + 1);
        *pPrescaler = newPrescaler;
    }
}

/**
 * @brief Callback unica di input capture per TIM3 e TIM4.
 *        Verifica se si tratta di TIM3_CH1 o TIM4_CH2 e agisce di conseguenza.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* === Se si tratta di TIM3_CH1 (Encoder1_CHA) === */
    if ((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (g_Encoder1.icLastCapture != 0)
        {
            uint32_t delta = 0;
            if (capture >= g_Encoder1.icLastCapture)
                delta = capture - g_Encoder1.icLastCapture;
            else
                delta = (0x10000UL - g_Encoder1.icLastCapture) + capture;

            if (delta > 0)
            {
                g_Encoder1.icVelocityTPS = (float)s_TIM3ClockFreq / (float)delta;
            }
            else
            {
                g_Encoder1.icVelocityTPS = 0.0f;
            }

            /* Esegui cambio dinamico del prescaler se necessario */
            ENC_IC_AdjustPrescaler(htim, &s_TIM3Prescaler, &s_TIM3ClockFreq, delta);
        }

        g_Encoder1.icLastCapture = capture;
    }

    /* === Se si tratta di TIM4_CH2 (Encoder2_CHA) === */
    else if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        if (g_Encoder2.icLastCapture != 0)
        {
            uint32_t delta = 0;
            if (capture >= g_Encoder2.icLastCapture)
                delta = capture - g_Encoder2.icLastCapture;
            else
                delta = (0x10000UL - g_Encoder2.icLastCapture) + capture;

            if (delta > 0)
            {
                g_Encoder2.icVelocityTPS = (float)s_TIM4ClockFreq / (float)delta;
            }
            else
            {
                g_Encoder2.icVelocityTPS = 0.0f;
            }

            ENC_IC_AdjustPrescaler(htim, &s_TIM4Prescaler, &s_TIM4ClockFreq, delta);
        }

        g_Encoder2.icLastCapture = capture;
    }
}
