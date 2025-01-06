#include "motors.h"
#include "main.h"

// ------------------------------------------------------------------------
//        VARIABILI GLOBALI ENCODER
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
//           FUNZIONI PER IL CONTROLLO PWM (TIM8)
// ------------------------------------------------------------------------

void MotorControl_StartPWM(void)
{
    // Avvia i canali “normali”
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // Avvia i canali “complementari” (solo CH1N, CH2N e CH3N)
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    //HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

void MotorControl_StopPWM(void)
{
    // Ferma i canali “normali”
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    //HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);

    // Ferma i canali “complementari”
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    //HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
}


void MotorControl_SetMotors(float speedMotor1,
                            float speedMotor2,
                            bool  enableM1,
                            bool  enableM2,
                            bool  auxM1,
                            bool  auxM2)
{
    // Saturazione
    if (speedMotor1 >  1.0f) speedMotor1 =  1.0f;
    if (speedMotor1 < -1.0f) speedMotor1 = -1.0f;
    if (speedMotor2 >  1.0f) speedMotor2 =  1.0f;
    if (speedMotor2 < -1.0f) speedMotor2 = -1.0f;

    // Leggi il periodo dal registro di TIM8 (AutoReload)
    float period = (float)__HAL_TIM_GET_AUTORELOAD(&htim8);

    // Mappiamo [-1..+1] => [0..period]
    //  -1 => 0
    //   0 => period/2
    //  +1 => period
    float ccrValMotor1 = (speedMotor1 + 1.0f) * 0.5f * period;
    float ccrValMotor2 = (speedMotor2 + 1.0f) * 0.5f * period;

    // Aggiorna i CCR (duty cycle)
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)ccrValMotor1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)ccrValMotor1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)ccrValMotor2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)ccrValMotor2);

    // Gestione pin di Enable
    HAL_GPIO_WritePin(PB12_MOT1_EN_GPIO_Port, PB12_MOT1_EN_Pin,
                      enableM1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PD3_MOT2_EN_GPIO_Port, PD3_MOT2_EN_Pin,
                      enableM2 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Imposta i pin AUX
    MotorControl_SetAux(auxM1, auxM2);
}

void MotorControl_SetAux(bool auxM1, bool auxM2)
{
    HAL_GPIO_WritePin(PE8_MOT1_AUX_GPIO_Port, PE8_MOT1_AUX_Pin,
                      auxM1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PE9_MOT2_AUX_GPIO_Port, PE9_MOT2_AUX_Pin,
                      auxM2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorControl_SetDeadTime(uint32_t deadTimeTicks)
{
    // Ferma il timer
    MotorControl_StopPWM();
    HAL_TIM_Base_Stop(&htim8);

    TIM_BreakDeadTimeConfigTypeDef sBDTR_Config = {0};
    sBDTR_Config.DeadTime = deadTimeTicks;
    // Se usi "Break" e "LockLevel" impostali coerentemente a CubeIDE
    // sBDTR_Config.OffStateRunMode  = TIM_OSSR_DISABLE;
    // sBDTR_Config.OffStateIDLEMode = TIM_OSSI_DISABLE;
    // sBDTR_Config.LockLevel        = TIM_LOCKLEVEL_OFF;
    // sBDTR_Config.BreakState       = TIM_BREAK_DISABLE;
    // sBDTR_Config.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    // sBDTR_Config.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBDTR_Config) != HAL_OK)
    {
        // Error handler
    }

    // Riavvia il timer + PWM
    HAL_TIM_Base_Start(&htim8);
    MotorControl_StartPWM();
}
