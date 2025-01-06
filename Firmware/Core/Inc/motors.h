#ifndef __MOTORS_H
#define __MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   // Se qui dentro c'è la definizione di htim8, htim1, htim2, ecc.
#include "tim.h"

/* ------------------------------------------------------------------------
 *                       PROTOTIPI FUNZIONI PWM (TIM8)
 * ------------------------------------------------------------------------ */

/**
 * @brief Avvia il PWM su TIM8 (tutti i canali necessari).
 */
void MotorControl_StartPWM(void);

/**
 * @brief Ferma il PWM su TIM8 (tutti i canali interessati).
 */
void MotorControl_StopPWM(void);

/**
 * @brief Imposta la velocità (e direzione) di due motori pilotati dal timer TIM8.
 *        - range speedMotor: [-1.0 .. +1.0]
 *        -  0.0  => duty ~ 50%, motore fermo
 *        - +1.0 => duty ~ 100%, massima velocità (senso A)
 *        - -1.0 => duty ~ 0%,   massima velocità (senso B)
 *
 * @param speedMotor1 Valore di velocità per il motore 1.
 * @param speedMotor2 Valore di velocità per il motore 2.
 * @param enableM1    true=abilita motore1, false=disabilita
 * @param enableM2    true=abilita motore2, false=disabilita
 * @param auxM1       true=pin AUX=HIGH, false=LOW
 * @param auxM2       true=pin AUX=HIGH, false=LOW
 */
void MotorControl_SetMotors(float speedMotor1,
                            float speedMotor2,
                            bool  enableM1,
                            bool  enableM2,
                            bool  auxM1,
                            bool  auxM2);

/**
 * @brief Imposta unicamente i pin AUX dei due motori.
 *
 * @param auxM1 true => HIGH, false => LOW
 * @param auxM2 true => HIGH, false => LOW
 */
void MotorControl_SetAux(bool auxM1, bool auxM2);

/**
 * @brief Imposta il DeadTime (TIM8) a runtime, fermando il timer e riavviandolo.
 *
 * @param deadTimeTicks Valore di DeadTime (0..255 a seconda della config)
 */
void MotorControl_SetDeadTime(uint32_t deadTimeTicks);


#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H */
