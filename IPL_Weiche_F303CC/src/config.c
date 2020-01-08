/* Includes ------------------------------------------------------------------*/
#include <config.h>

/* Functions -----------------------------------------------------------------*/
void RCC_Config(void){
	/* PLL Config */
	RCC->CR &= ~(RCC_CR_PLLON); 																	// Deactivate PLL
	while(RCC->CR & RCC_CR_PLLRDY){}																// Wait for PLL to stop
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL16;										// Set PLL to input HSI/2, Multiplicator = 16 => HCLK=64MHz
	RCC->CR |= RCC_CR_PLLON; 																		// Activate PLL
	while(!(RCC->CR & RCC_CR_PLLRDY)); 																// Wait for PLL to lock

	/*FLASH wait states */
	FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk);															// Reset Flash Wait states
	FLASH->ACR |= 0b010 << FLASH_ACR_LATENCY_Pos;													// Set Flash wait states to 2

	/*SysClock anpassen */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 																// PreDiv for ABP1 /2 (ABP1 36MHz max)
	RCC->CFGR |= RCC_CFGR_SW_PLL;																	// Set PLL as Sysclock
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){} 										// Wait for switch to PLL as clock source

  	SystemCoreClockUpdate();

	/* Peripherie Clock */

  	/* Timer */
  	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 Clock
  	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable TIM4 Clock
  	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable TIM4 Clock
  	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // Enable TIM4 Clock
  	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8 Clock

	/* GPIO */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //GPIO A
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //GPIO B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //GPIO C

	/* ADC */
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; //ADC12
	/* Other */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Clock to SYSCFG


}


void GPIO_Config(void){

	/* ########## GPIO MODES ##########
	 *GPIOx -> MODER
	 *
	 * GPIOx_MODER_INPUT
	 * GPIOx_MODER_OUTPUT
	 * GPIOx_MODER_ALT
	 * GPIOx_MODER_ANALOG
	 */

	/* Inputs */
	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER1_Pos); 	// PA1 Pos1 Input
	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER2_Pos); 	// PA2 Pos2 Input



	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER5_Pos); 	// PA5 DIP-Switch 1 VOERST NOCH LD2
	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER6_Pos); 	// PA6 DIP-Switch 2
	GPIOA->MODER &= ~(0b11 << GPIO_MODER_MODER7_Pos); 	// PA7 DIP-Switch 3
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER0_Pos); 	// PB0 DIP-Switch 4
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER2_Pos); 	// PB2 DIP-Switch 5
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER10_Pos); 	// PB10 DIP-Switch 6
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER11_Pos); 	// PB11 DIP-Switch 7
	GPIOB->MODER &= ~(0b11 << GPIO_MODER_MODER12_Pos); 	// PB12 DIP-Switch 8

	/* Outputs */

	GPIOA->MODER |= GPIOx_MODER_OUTPUT<<GPIO_MODER_MODER3_Pos;		// PA3 nSleep
	GPIOA->MODER &= ~(GPIO_MODER_MODER15_Msk);
	GPIOA->MODER |= GPIOx_MODER_OUTPUT<<GPIO_MODER_MODER15_Pos;		// PA15 H-Bride 1A
	GPIOB->MODER &= ~(GPIO_MODER_MODER3_Msk);
	GPIOB->MODER |= GPIOx_MODER_OUTPUT<<GPIO_MODER_MODER3_Pos;		// PB3 H-Bride 1B
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_Msk);
	GPIOB->MODER |= GPIOx_MODER_OUTPUT<<GPIO_MODER_MODER4_Pos;		// PB4 H-Bride 2B
	GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk);
	GPIOB->MODER |= GPIOx_MODER_OUTPUT<<GPIO_MODER_MODER8_Pos;		// PB8 H-Bride 2A

	/* Analog */
	GPIOA->MODER |= GPIOx_MODER_ANALOG<<GPIO_MODER_MODER4_Pos;	// PA4 Fader

	/* Alternate functions */
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER0_Pos; 	// PA0 DCC Input

	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER11_Pos;	// PA11 LED1 PWM
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER12_Pos;	// PA12 LED2 PWM

//	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER15_Pos;	// PA15 H-Bride 1A
//	GPIOB->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER3_Pos;		// PB3 H-Bride 1B
//	GPIOB->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER4_Pos;		// PB4 H-Bride 2B
//	GPIOB->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER8_Pos;		// PB8 H-Bride 2A

	/* UART */
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER8_Pos;		// PA8 USART_CK
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER9_Pos;		// PA9 USART_TX
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER10_Pos;	// PA10 USART_RX

	/* SWD */
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER13_Pos;	// PA13 SWDIO
	GPIOA->MODER |= GPIOx_MODER_ALT<<GPIO_MODER_MODER14_Pos;	// PA14 SWCLK

	/* ########## GPIO Output Types ##########
	 * GPIOx -> OTYPER
	 *
	 * GPIOx_OTYPER_PP
	 * GPIOx_OTYPER_OD
	 */

	/* ########## GPIO Output Speed ##########
	 * GPIOx -> OSPEEDR
	 *
	 * GPIOx_OSPEEDR_LOW
	 * GPIOx_OSPEEDR_MED
	 * GPIOx_OSPEEDR_HIGH
	 */

	/* ########## GPIO Pull-up/-down ##########
	 * GPIOx -> PUPDR
	 *
	 * GPIOx_PUPDR_NO
	 * GPIOx_PUPDR_PU
	 * GPIOx_PUPDR_PD
	 */
	GPIOA->PUPDR	 |= GPIOx_PUPDR_PD << GPIO_PUPDR_PUPDR1_Pos | GPIOx_PUPDR_PD << GPIO_PUPDR_PUPDR2_Pos;			 //Pulldown for PA1/2 (ButtonL_R)
	GPIOA->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR5_Pos; 	// PA5 DIP-Switch 1 VOERST NOCH LD2
	GPIOA->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR6_Pos; 	// PA6 DIP-Switch 2
	GPIOA->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR7_Pos; 	// PA7 DIP-Switch 3
	GPIOB->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR0_Pos; 	// PB0 DIP-Switch 4
	GPIOB->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR2_Pos; 	// PB2 DIP-Switch 5
	GPIOB->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR10_Pos; // PB10 DIP-Switch 6
	GPIOB->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR11_Pos; // PB11 DIP-Switch 7
	GPIOB->PUPDR |= GPIOx_PUPDR_PD<<GPIO_PUPDR_PUPDR12_Pos; // PB12 DIP-Switch 8

	/* ########## GPIO Alternate funtion ########## */
	/* Trigger für Timer 2 */
	//GPIOA->AFR[0] |= 1<<GPIO_AFRL_AFRL0_Pos;	// PA0 AF1 TIM2 Ext.Trig.
	/* PWM für LEDs */
	GPIOA->AFR[1] |= 10<<GPIO_AFRH_AFRH3_Pos;	// PA11 AF10 TIM4_CH1
	GPIOA->AFR[1] |= 10<<GPIO_AFRH_AFRH4_Pos;	// PA12 AF10 TIM4_CH2
	/* PWM für Motor */
//	GPIOA->AFR[1] |= 2<<GPIO_AFRH_AFRH7_Pos;	// PA15 AF2 TIM8_CH1
//	GPIOB->AFR[0] |= 4<<GPIO_AFRL_AFRL3_Pos;	// PB3 AF4 TIM8_CH1N
//	GPIOB->AFR[1] |= 10<<GPIO_AFRH_AFRH0_Pos;	// PB8 AF10 TIM8_CH2
//	GPIOB->AFR[0] |= 4<<GPIO_AFRL_AFRL4_Pos;	// PB4 AF4 TIM8_CH2N

	/* UART */
	GPIOA->AFR[1] |= 7<<GPIO_AFRH_AFRH0_Pos;	// PA8 AF7 USART_CK
	GPIOA->AFR[1] |= 7<<GPIO_AFRH_AFRH1_Pos;	// PA9 AF7 USART_TX
	GPIOA->AFR[1] |= 7<<GPIO_AFRH_AFRH2_Pos;	// PA10 AF7 USART_RX

	/* ########## Reset Pins to Default ##########
	 * Port Reset		GPIO<port>->ODR &= ~(0xFFFF)
	 * Port Set			GPIO<port>->ODR |= 0xFFFF
	 * Pin Reset		GPIOA->ODR &= ~(GPIO_ODR_<pin>);
	 * Pin Set			GPIOA->ODR |= GPIO_ODR_<pin>;
	 */

}



void TIM_Config(void){

	/* Timer 2 */
	TIM2->CR1 &= ~TIM_CR1_CEN;					// disable TIM2
	TIM2->ARR = DCC_SAMPLEPOINT; 				// set counter value
	TIM2->PSC = (SystemCoreClock/1000000)-1;	// set prescaler value (usec)
	TIM2->CR1 |= TIM_CR1_OPM; 					// Enable one-pulse Mode
	TIM2->DIER |= TIM_DIER_UIE; 				// Enable Interrupt on Update
//	TIM2->SMCR |= 0b111 << TIM_SMCR_TS_Pos;		// External Trigger selected
//	TIM2->SMCR |= 0b0110 << TIM_SMCR_SMS_Pos;	// Slave Mode = Triggered Mode
//	TIM2->CR1 |= TIM_CR1_CEN;					// enable Timer2
	NVIC_EnableIRQ(TIM2_IRQn);					// Enable NVIC on TIM2

	/* Timer 4 LED PWM */
	TIM4->CCMR1		 = 0x6060;					//PWM MODE 1, reference manual->CNT<CCRx HIGH
	TIM4->CCER		 = 0x0011;					//Chanels ON
	TIM4->PSC		 = (SystemCoreClock/1000)-1;//alle 0.1ms
	TIM4->ARR		 = LED_ARR;					//Max counternum.
	TIM4->CCR1		 = LED_ARR/2;				//Ontime LEDon (undefinierter zustand) BEIDE LEDs BLINKEN
	TIM4->CCR2		 = LED_ARR/2;				//Ontime LEDon (undefinierter zustand) BEIDE LEDs BLINKEN
	TIM4->CR1		 = 0x0001;					//COUNTER (TIM4) ENABLE

	/* Timer 8 Motor PWM */
	TIM8->CCMR1		= 0x00003030;				//PWM Mode1 CH1/2 TOGGLE_MODE PA7/8/9/12
	TIM8->CCER		= 0x00000044;				//CH1/2&1n/2n enable, polarity
	TIM8->BDTR		= 0x0000CC00;				//MOE/AOE/OSSR/OSSI ON
	TIM8->PSC		= (SystemCoreClock/1000)-1;	//1ms
	TIM8->ARR		= MOTOR_MS-1;
	TIM8->CCR1		= ((TIM8->ARR)+1)/2;
	TIM8->CCR2 		= 0;
//	TIM8->CR1		= 0x00000001;				//counter enable

	/* Delay Timer 6 */
	TIM6->ARR = 0; 								//set counter value
	TIM6->PSC = (SystemCoreClock/1000)-1;		//prescaler for ARR in ms
	TIM6->CR1 |= TIM_CR1_OPM; 					//Enable one-pulse Mode
	TIM6->DIER |= TIM_DIER_UIE; 				//Enable Interrupt on Update
	TIM6->CR1 &= ~TIM_CR1_CEN;					//disable Timer

	/* Motor Timer 7 */
	TIM7->ARR = 399; 							// set counter value
	TIM7->PSC = (SystemCoreClock/100000)-1;		// prescaler for ARR in ms
	TIM7->DIER |= TIM_DIER_UIE; 				// Enable Interrupt on Update
	TIM7->CR1 |= TIM_CR1_CEN;					// enable Timer
	NVIC_EnableIRQ(TIM7_IRQn);
}

void ADC_Config(void){

	ADC12_COMMON->CCR |= 0x00010000; 		// Synchroner ADC-Clock ,Vorteiler 1
	ADC2->SQR1 |= 0x00000040; 				// 1st conv. in regular sequence: Channel 1 (PA4)
	ADC2->CR &=~0x30000000; 		    	// Voltage regulator: Intermediate state (0b11 << 28), 10 resetstate
	ADC2->CR |= 0x50000000; 				// Voltage regulator: Enabled (0b01 << 28) DifferentialMode
	ADC2->SMPR1 |= 0x00000000;				// 601.5 clockcycles
	for (volatile int x = 0; x < 60; x++){} // Warte 10 us
	ADC2->CR |= 0x80000000; 				// Kalibriere den ADC
	while ((ADC2->CR & 0x80000000) != 0){}  // Warte bis Kalibrierung abgeschlossen
	ADC2->IER = 0x0080; 					// INTERRUPT enable analog watchdog1
	ADC2->TR1 = 0xFFD00014;					// 0x007C [H]->124->bei 100mA �ber 1 OHM
	ADC2->CFGR = 0x04C03000;				// Watchdog Enable on regular channels(23),AWD1CH 2(29-26),continuous (13),overrunmode(12)

	ADC2->CR |= 0x00000001; 		 		// Enable ADC
	while((ADC2->ISR & 0x00000001) == 0){} 	// Warte bis ADC bereit
	ADC2->CR |= 0x00000004;					// start regular conversation
	NVIC_EnableIRQ (ADC1_2_IRQn);

}

void EXTI_Config(void){

	/* EXTI für PA0, DCC */
	SYSCFG->EXTICR[0] 		= 0; // PA0
	EXTI->IMR				|= EXTI_IMR_MR0;
	EXTI->RTSR				|= EXTI_RTSR_RT0;
	NVIC_EnableIRQ(EXTI0_IRQn);

	/* EXTI für PA1, Button links */
	SYSCFG->EXTICR[0] 		= 0; // PA1
	EXTI->IMR				|= EXTI_IMR_MR1;
	EXTI->FTSR				|= EXTI_FTSR_FT1;
	NVIC_EnableIRQ(EXTI1_IRQn);

	/* EXTI für PA2, Button rechts */
	SYSCFG->EXTICR[0] 		= 0; // PA_2
	EXTI->IMR				|= EXTI_IMR_MR2;
	EXTI->FTSR				|= EXTI_FTSR_FT2;
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);



}
