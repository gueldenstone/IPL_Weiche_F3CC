/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* TypeDefs ------------------------------------------------------------------*/
typedef enum{WF_Preamble, WF_Lead0, WF_Byte, WF_Trailer}TypeDefRecstate;
typedef enum{Left,Right}TypeDefPosition;
typedef struct{
	uint8_t dcc_address;
	uint8_t turnout_address;
	TypeDefPosition direction;
}TypeDefPackage;

/* Defines -------------------------------------------------------------------*/
//DCC-Decode
#define DCC_SAMPLEPOINT 86
#define TURNOUT1_SET_LEFT	EXTI->SWIER |= 0x2
#define TURNOUT1_SET_RIGHT	EXTI->SWIER |= 0x4

/* Weiche */
#define H_BRIDGE_OFF	(GPIOA->BRR |= GPIO_BRR_BR_3) //H_Bridge OFF PA3
#define H_BRIDGE_ON 	(GPIOA->BSRR|= GPIO_BSRR_BS_3) //H_Bridge ON  PA3
#define MOTORPWM_OFF 	TIM8->CCER&=~0x00000011
#define MOTOR_LEFT 		TIM8->CCER=0x0000005F
#define MOTOR_RIGHT		TIM8->CCER=0x00000055
#define MOTOR_MS		500

/* LEDs */
#define LED_ARR		(200)
#define blinkonr	TIM4->CCR1=LED_ARR/2
#define blinkonl	TIM4->CCR2=LED_ARR/2
#define LEDonr		TIM4->CCR1=1000
#define LEDonl		TIM4->CCR2=1000
#define LEDoffr		TIM4->CCR1=0000
#define LEDoffl		TIM4->CCR2=0000

/* Buttons & Poti */
#define POTI ADC2->DR //4095 links;0 rechts
#define DELTA_MEAS_TIME 250		// Zeit (in ms) zwischen zwei Messungen (bis Stillstand erkannt wird)

/* Prototypes -----------------------------------------------------------------*/
void Error_Handler(void);
uint8_t get_address(void);
void checkpos(void);
void delay(uint32_t ms);
void RESET_Function(void);

#endif /* __MAIN_H */
