#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f0xx.h"                  // Device header
#include "motor.h"




//PA4 = Enable/ PWM
//PA5/PA8 = IN1 and IN2/ Direction ctrl
/* 
-----------------------------------------------------------------------------------
--------------------------
 *  Global Variable Declarations
 *  
-----------------------------------------------------------------------------------
--------------------------
 */
volatile uint32_t debouncer;
volatile int position = 0;
volatile uint32_t button_count = 0;
/* 
-----------------------------------------------------------------------------------
--------------------------
 *  Miscellaneous Core Functions
 *  
-----------------------------------------------------------------------------------
--------------------------
 */



void close_blinds(void) {

	button_count++;
	// Direction control
	if (position == 0){
	GPIOA->ODR |= (1 << 5); // closed CW
  GPIOA->ODR &= ~(1 << 8);	
	GPIOA->ODR |= (1 << 4); //Enable PA4: ON
		 
	}
	if (position == 1){
	  GPIOA->ODR &= ~(1 << 5);
		GPIOA->ODR |= (1 << 8); // open CCW
		GPIOA->ODR |= (1 << 4); //Enable PA4: ON


	}
	position = !position;

	return;
}


void TIM2_IRQHandler (void){ //TIM2 Handler
	
	GPIOC->ODR ^= (GPIO_ODR_8 | GPIO_ODR_9);
	GPIOA->ODR &= ~(GPIO_ODR_4); //Disable PA4: OFF
	TIM2->SR &= ~TIM_SR_UIF; 
	TIM2->CR1 &= ~TIM_CR1_CEN;
	return;
}


void transmitLetter(char c){
	while((USART3->ISR & USART_ISR_TXE) == 0);
	USART3->TDR = c;
	return;
}

void transmitString(char* s){
	int indx = 0;
	while(s[indx] != 0){
		transmitLetter(s[indx]);
		indx++;
	}
	return;
}
volatile int calledTimes = 0;

void callMe(void){
	calledTimes++;
}

volatile int success = 0;
volatile int transmittedATString = 0;
volatile int transmittedAskModeString = 0;
volatile int transmittedSetModeString = 0;
volatile int transmittedConnectedWiFiString = 0;
volatile int transmittedAskWiFiString = 0;
volatile int transmittedECsString = 0;
volatile int transmittedS80String = 0;
volatile int transmittedPrepReqString = 0;
volatile int transmittedDataString = 0;
volatile int transmittedCloseConString = 0;

int main(void)
{
	HAL_Init();
  success = 0;
  transmittedATString = 0;
  transmittedAskModeString = 0;
  transmittedSetModeString = 0;
  transmittedConnectedWiFiString = 0;
  transmittedAskWiFiString = 0;
  transmittedECsString = 0;


	//	// New motor init 	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  	
  GPIOA->MODER &= 0xFFFCF3FF; // clear PA5, PA8 bits,
	GPIOA->MODER |= (1 << 10) | (1 << 16);
	GPIOA->MODER |= (1 << 8);   // PA4 to output
	GPIOA->ODR |= (1 << 5); // initialize to CW
  GPIOA->ODR &= ~(1 << 8);
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  
	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->APB1ENR|= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 7999;
	TIM2->ARR = 5999;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	GPIOC->MODER &= ~GPIO_MODER_MODER8;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER &= ~GPIO_MODER_MODER9;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER4; // PC4 TX
	GPIOC->MODER |= GPIO_MODER_MODER4_1;
	GPIOC->MODER &= ~GPIO_MODER_MODER5; // PC5 RX
	GPIOC->MODER |= GPIO_MODER_MODER5_1;
	
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5); // Configure mode to output for the LEDS
	GPIOC->AFR[0] |= 0x00110000;
	
	uint16_t usartdiv = (int)(HAL_RCC_GetHCLKFreq()/115200);
	USART3->BRR = usartdiv;
	
	USART3->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	USART3->CR1 |= USART_CR1_UE;
	
	
	char lastTen[10];
	char dataVal;
	char databuff[256];
	volatile uint8_t databuffindex = 0;
	char ATString[] = "AT\r\n";
	char AskModeString[] = "AT+CWMODE?\r\n";
	char SetModeString[] = "AT+CWMODE=1\r\n";
	char ConnectWiFiString[] = "AT+CWJAP=\"999\",\"BurtMacklin\"\r\n";
//	char ConnectWiFiString[] = "AT+CWJAP=999,BurtMacklin";
//	char ConnectWiFiString[] = "AT+CWJAP=[999],[BurtMacklin]";
//	char ConnectWiFiString[] = "AT+CWLAP";
	char AskIPAddress[] = "AT+CIFSR\r\n";
	char ECs[] = "AT+CIPMUX=1\r\n";
	char s80[] = "AT+CIPSERVER=1,80\r\n";
	char prepReq[] = "AT+CIPSEND=0,5\r\n";
	char dataStr[] = "hello\r\n";
	char closeCon[] = "AT+CIPCLOSE=0\r\n";
	while (1) {
		while((USART3->ISR & USART_ISR_RXNE) == 0);
		dataVal = USART3->RDR;
		databuff[databuffindex] = dataVal;
		lastTen[0] = lastTen[1];
		lastTen[1] = lastTen[2];
		lastTen[2] = lastTen[3];
		lastTen[3] = lastTen[4];
		lastTen[4] = lastTen[5];
		lastTen[5] = lastTen[6];
		lastTen[6] = lastTen[7];
		lastTen[7] = lastTen[8];
		lastTen[8] = lastTen[9];
		lastTen[9] = dataVal;
		databuffindex++;	
		if(lastTen[6] == 'I' && lastTen[7] == 'P' && lastTen[8] == '\r' && lastTen[9] == '\n' && transmittedECsString == 0){
			HAL_Delay(500);
			transmitString(ECs);
			transmittedECsString = 1;
		}
		else if(lastTen[6] == '6' && lastTen[7] == '9'){
			callMe();
			TIM2->CR1 |= TIM_CR1_CEN;
//			TIM2->CNT = 0;
			close_blinds();
		}
		else if(lastTen[6] == 'O' && lastTen[7] == 'K' && lastTen[8] == '\r' && lastTen[9] == '\n' && transmittedS80String == 0){
			HAL_Delay(500);
			transmitString(s80);
			transmittedS80String = 1;
		}
		else if(lastTen[5] == '9' && lastTen[6] == '\r' && lastTen[7] == '\n' && lastTen[8] == '\r' && lastTen[9] == '\n' && transmittedPrepReqString == 0){
			HAL_Delay(500);
			transmitString(prepReq);
			transmittedPrepReqString = 1;
		}
		else if(lastTen[5] == 'K' && lastTen[6] == '\r' && lastTen[7] == '\n' && lastTen[8] == '>' && lastTen[9] == 0x20 && transmittedDataString == 0){
			HAL_Delay(500);
			transmitString(dataStr);
			transmittedDataString = 1;
		}
		else if(lastTen[2] == 'E' && lastTen[3] == 'N' && lastTen[4] == 'D' && lastTen[5] == 0x20 && lastTen[6] == 'O' && lastTen[7] == 'K' && lastTen[8] == '\r' && lastTen[9] == '\n' && transmittedCloseConString == 0){
			HAL_Delay(500);
			transmitString(closeCon);
			transmittedCloseConString = 1;
		}
//		else if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'K' && lastTen[1] == 'O' && transmittedAskModeString == 0){
//			success = 1;
//			HAL_Delay(500);
//			transmitString(AskModeString);
//			transmittedAskModeString = 1;
//		}
//		if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'y' && lastTen[1] == 'd' && lastTen[0] == 'a' && transmittedATString == 0){
//			HAL_Delay(500);
//			transmitString(ATString);
//			transmittedATString = 1;
//		
//		}
//		else if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'K' && lastTen[1] == 'O' && transmittedAskModeString == 0){
//			success = 1;
//			HAL_Delay(500);
//			transmitString(AskModeString);
//			transmittedAskModeString = 1;
//		}
//		else if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'K' && lastTen[1] == 'O' && transmittedSetModeString == 0){
//			success = 1;
//			HAL_Delay(500);
//			transmitString(SetModeString);
//			transmittedSetModeString = 1;
//			transmittedAskModeString = 0;
//		}
//		else if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'K' && lastTen[1] == 'O' && transmittedConnectedWiFiString == 0){
//			success = 1;
//			HAL_Delay(500);
//			transmitString(ConnectWiFiString);
//			transmittedConnectedWiFiString = 1;
//		}
//		else if(lastTen[4] == '\n' && lastTen[3] == '\r' && lastTen[2] == 'K' && lastTen[1] == 'O' && transmittedAskWiFiString == 0){
//			success = 1;
//			HAL_Delay(500);
//			transmitString(AskIPAddress);
//			transmittedAskWiFiString = 1;
//		}
	}

}
