/**
 ******************************************************************************
 * @file    interrupts.c
 * @author  lukas
 * @version V1.0
 * @date    02-Feb-2015
 * @brief   Default Interrupt Service Routines.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <interrupts.h>
#include "stm32f3xx.h"


/* Globale Variablen */
extern volatile uint8_t package[3];
extern volatile _Bool received;
extern volatile int set;

/* Modul-Variablen */
_Bool stepdirection=0, stepenable=0;

/*************************************************************************
Function: setstepdirection(_Bool direction)
Purpose:  Setting the direction for the motor
Input:    Boolean for fwd or bwd motion (0=bwd, 1=fwd)
Returns:  none
Comment:  is called by other modules
**************************************************************************/
void setstepdirection(_Bool direction){
	stepdirection = direction;
}

/*************************************************************************
Function: setstepenable(_Bool enable)
Purpose:  enables the rotation of the motor
Input:    Boolean for go or stop (0=stop, 1=go)
Returns:  none
Comment:  is called by other modules
**************************************************************************/
void setstepenable(_Bool enable){
	stepenable = enable;
}

/*************************************************************************
Function: TIM7_IRQHandler(void)
Purpose:  state machine for setting the GPIOs for the motor-driving pattern
Input:    none
Returns:  none
Comment:
**************************************************************************/
void TIM7_IRQHandler(void){
	volatile static uint8_t outputpattern;
	if(!stepenable){
		GPIOA->BSRR |= GPIO_BSRR_BR_15;	// AIN1 = off
		GPIOB->BSRR |= GPIO_BSRR_BR_8; 	// AIN2 = off
		GPIOB->BSRR |= GPIO_BSRR_BR_3; 	// BIN1 = off
		GPIOB->BSRR |= GPIO_BSRR_BR_4; 	// BIN2 = off
		outputpattern = 0;
	} else{

		// noch mit BSRR lösen!
		switch(outputpattern){
		case 0:
			GPIOB->BSRR |= GPIO_BSRR_BR_8; 	// AIN2 = off
			GPIOB->BSRR |= GPIO_BSRR_BR_3; 	// BIN1 = off
			GPIOA->BSRR |= GPIO_BSRR_BS_15;	// AIN1 = on
			GPIOB->BSRR |= GPIO_BSRR_BS_4; 	// BIN2 = on
			break;
		case 1:
			GPIOB->BSRR |= GPIO_BSRR_BR_8; 	// AIN2 = off
			GPIOB->BSRR |= GPIO_BSRR_BR_4; 	// BIN2 = off
			GPIOA->BSRR |= GPIO_BSRR_BS_15;	// AIN1 = on
			GPIOB->BSRR |= GPIO_BSRR_BS_3; 	// BIN1 = on
			break;
		case 2:
			GPIOA->BSRR |= GPIO_BSRR_BR_15;	// AIN1 = off
			GPIOB->BSRR |= GPIO_BSRR_BR_4; 	// BIN2 = off
			GPIOB->BSRR |= GPIO_BSRR_BS_8; 	// AIN2 = on
			GPIOB->BSRR |= GPIO_BSRR_BS_3; 	// BIN1 = on
			break;
		case 3:
			GPIOA->BSRR |= GPIO_BSRR_BR_15;	// AIN1 = off
			GPIOB->BSRR |= GPIO_BSRR_BR_3; 	// BIN1 = off
			GPIOB->BSRR |= GPIO_BSRR_BS_8; 	// AIN2 = on
			GPIOB->BSRR |= GPIO_BSRR_BS_4; 	// BIN2 = on
			break;
		default:
			break;
		}
		if(stepdirection){
			outputpattern = (outputpattern+1)&0x3;		// Bis vier Zählen und wieder von vorne anfangen
		}
		else{
			outputpattern = (outputpattern-1)&0x3;
		}
	}
	/* ISR finished */
	TIM7->SR &= ~TIM_SR_UIF;
}

/*************************************************************************
Function: TIM2_IRQHandler(void)
Purpose:  reading the Bit from DCC-Stream and packing it into package
Input:    none
Returns:  none
Comment:  uses global variable package[]
**************************************************************************/
void TIM2_IRQHandler(void){
	static int8_t t=0, i=8;
	static TypeDefRecstate recstate;
	static uint8_t byte;
	static _Bool bit=0;

	if(!received){								// checken, ob das letzte empfangene Packet ausgewertet wurde
		bit = !(GPIOA->IDR & 1);				// Status PA0 auslesen -> inverser Wert ist Bit
		switch(recstate){						// Recstate abfragen
		case WF_Preamble:						// Warten auf die Preambel
			if((bit==1) && (t<12)) t++;			// Preambel noch nicht vorbei
			if((bit==1) && (t>=12)){			// Preambel vorbei
				recstate=WF_Lead0;				// Nächster Status: Warten auf das Trennbit
			}
			if(bit==0) t=0;						// Synchronisierungsfehler, reset
			break;
		case WF_Lead0:							// Auf Trennbit warten
			if (bit==0) recstate=WF_Byte;		// Falls Trennbit erkannt, erstes Byte auslesen
			break;
		case WF_Byte:							// Auf Byte warten
			package[byte] |= bit<<(i-1);		// Bitstelle des Bytes auslesen und speichern
			i--;
			if(i<=0){							// Ende des Bytes erreicht
				recstate=WF_Trailer;			// Warten auf das nächste Trennbit
				byte++;							// nächstes Byte auswählen
				i=8;							// Bitstelle zurücksetzten
			}
			break;
		case WF_Trailer:						// Auf Trenn- oder Stopbit warten
			if(bit==0) recstate=WF_Byte;		// Trennbit erkannt, nächstes Byte auslesen
			if(bit==1 && byte>=2){				// Stopbit erkannt, rücksetzten der Indizes
				received=1;						// main() signalisieren, dass package bereit steht
				byte=0;
				i=8;
				t=0;
				recstate=WF_Preamble;			// warten auf nächstes package
			}
			break;
		}
	}
	/* ISR finished */
	TIM2->SR &= ~TIM_SR_UIF;
}

void ADC1_2_IRQHandler(void){
	if (POTI<10)
	{
		ADC2->TR1 =0xFFFF0000;
		LEDoffl;
		LEDonr;
		set=3;
	}
	else if (POTI>3000)
	{
		ADC2->TR1 =0xFFFF0000;
		LEDoffr;
		LEDonl;
		set=3;
	}
	H_BRIDGE_OFF;
	setstepenable(0);

	/* ISR finished */
	NVIC_ClearPendingIRQ(ADC1_2_IRQn); //Interruptflag cleared
	ADC2->ISR		|= 0x0080;		   //ADC interruptflag cleared

}

void EXTI0_IRQHandler(void){
	TIM2->CR1 |= TIM_CR1_CEN;
	EXTI->PR |= EXTI_PR_PR0;
}

void EXTI1_IRQHandler(void){			//button links
	LEDoffr;
	blinkonl;
	H_BRIDGE_ON;
	setstepdirection(0);
	setstepenable(1);
	ADC2->TR1 = (0xFFFF<<16) | 0x000A;
	set=1;

	/* ISR finished */
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	EXTI->PR |= EXTI_PR_PR1;
}

void EXTI2_TSC_IRQHandler(void){			//button rechts
	LEDoffl;
	blinkonr;
	H_BRIDGE_ON;
	setstepdirection(1);
	setstepenable(1);
	ADC2->TR1 = (0xBB8<<16) | 0x0000;
	set=2;

	/* ISR finished */
	NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
	EXTI->PR |= EXTI_PR_PR2;
}

