/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Prototypes -----------------------------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
/* EXTI */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_TSC_IRQHandler(void);
/* TIM */
void TIM2_IRQHandler(void);
void TIM7_IRQHandler(void);
/* ADC */
void ADC1_2_IRQHandler(void);
/* public utility functions */
void setstepdirection(_Bool direction);
void setstepenable(_Bool enable);
#endif

