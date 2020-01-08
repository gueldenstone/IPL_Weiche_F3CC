/**
  ******************************************************************************
  * @author  Kevin Heidenreich & Lukas Güldemstein
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
//Includes
#include <stdio.h>
#include <stdlib.h>

//Private Includes
#include "main.h"
#include "config.h"
#include "interrupts.h"
#include "stm32f3xx.h"

//DCC-Variablen
volatile uint8_t package[3];
volatile _Bool received;

//Weiche Variablen
volatile int set=1;


int main(void){

	volatile TypeDefPackage rPackage;			// received Package
	volatile uint8_t dcc_address, parity;		// DCC-Adresse

	__disable_irq();				//Interrupts ausschalten

	/* Configuration */
	RCC_Config();					// Clock Config auf 72Mhz
	GPIO_Config(); 					// GPIO Config
	TIM_Config();					// Config Timer
	ADC_Config();					// ADC2 Config
	EXTI_Config();					// EXTI0 & 1 Config !! noch nicht richtig

	delay(100);
	/* Initialisierung/Kalibrierung */
	RESET_Function();				// Weichensteuerung initialisieren
	dcc_address = 162;				// Adresse auslesen
	__enable_irq();					// Interrupts einschalten

	//Schleife
	while(1){
		/* Position auslesen */
		checkpos();		//Refactor
		/* DCC Decode */
		if(received){
			rPackage.turnout_address=(package[1] & 0x6)>>1;		// Weichenadresse auslesen
			rPackage.direction=(package[1] & 0x1);				// Richtung auslesen
			/*	byte 1		byte 2
				10AAAAAA  	1AAA1BBR

				A=address
				B=turnout
				R=direction
			*/
			rPackage.dcc_address = ((package[0] & 0x3F)<<3) | ((package[1] & 0x70)>>4);	//DCC-Adresse auslesen
			//Pariät prüfen
			if((package[0]^package[1])==package[2]){
				parity = 1;
			}
			/* DCC Adresse prüfen */
			if(rPackage.dcc_address==dcc_address && parity){
				switch(rPackage.turnout_address){
				case 0:											// Weiche 0 angesprochen
					switch(rPackage.direction){
					case Left:
						TURNOUT1_SET_LEFT;
						break;
					case Right:
						TURNOUT1_SET_LEFT;
						break;
					}
					break;
				case 1:											// Weiche 1 angesprochen
					switch(rPackage.direction){
					case Left:
						TURNOUT1_SET_LEFT;
						break;
					case Right:
						TURNOUT1_SET_RIGHT;
						break;
					}
					break;
				case 2:											// Weiche 2 angesprochen
					switch(rPackage.direction){
					case Left:
						TURNOUT1_SET_LEFT;
						break;
					case Right:
						TURNOUT1_SET_RIGHT;
						break;
					}
					break;
				case 3:											// Weiche 3 angesprochen
					switch(rPackage.direction){
					case Left:
						TURNOUT1_SET_LEFT;
						break;
					case Right:
						TURNOUT1_SET_RIGHT;
						break;
					}
					break;
				case 4:											// Weiche 4 angesprochen
					switch(rPackage.direction){
					case Left:
						TURNOUT1_SET_LEFT;
						break;
					case Right:
						TURNOUT1_SET_RIGHT;
						break;
					}
					break;
				default:										// sonst fehlerhaft, break
					break;
				}

				while(set!=3){									// warten bis die Weiche vollständig gestellt wurde
					checkpos();
				}
				received=0;										// received flag zurücksetzten
				package[0] = 0;
				package[1] = 0;
				package[2] = 0;
			}
			else{
				received = 0;
				package[0] = 0;
				package[1] = 0;
				package[2] = 0;

			}
		}
	}
}

void checkpos(void){
	if ((set==1) || (set==2)){
		int fader_old=POTI;
		int fader_sub50, fader_add50;
		if(fader_old>50)
			fader_sub50 = fader_old-15;
		else
			fader_sub50 =0;
		fader_add50 = fader_old+15;
		delay(DELTA_MEAS_TIME);											// warte 1s
		if ((POTI>=fader_sub50) && (POTI<=fader_add50))
		{
			setstepenable(0);
			H_BRIDGE_OFF;
			blinkonl;
			blinkonr;
			set=3;
		}
	}

}

void delay(uint32_t ms){
	TIM6->ARR = ms;					// Setzte ARR auf Wert in ms
	TIM6->CR1 |= TIM_CR1_CEN;		// Enable Timer 6
	while(!(TIM6->SR&1)){}			// warte ab bis Timer 6 gezählt hat
	TIM6->SR &= ~TIM_SR_UIF;		// UIF zurücksetzten
}

uint8_t get_address(void){
	uint8_t address=0;
	address |= (GPIOA->IDR & GPIO_IDR_5)<<2;
	address |= (GPIOA->IDR & GPIO_IDR_6)<<0;
	address |= (GPIOA->IDR & GPIO_IDR_7)>>2;
	address |= (GPIOB->IDR & GPIO_IDR_0)<<4;
	address |= (GPIOB->IDR & GPIO_IDR_2)<<1;
	address |= (GPIOB->IDR & GPIO_IDR_10)>>8;
	address |= (GPIOB->IDR & GPIO_IDR_11)>>10;
	address |= (GPIOB->IDR & GPIO_IDR_12)>>12;
	return address;
}

void RESET_Function(void){
	H_BRIDGE_OFF;
	setstepenable(0);
}
