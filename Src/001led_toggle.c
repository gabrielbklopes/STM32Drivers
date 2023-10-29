/*
 * 001led_toggle.c
 *
 *  Created on: 7 de out de 2023
 *      Author: gabri
 */

#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"


void delay(void){
	for(uint32_t i=0; i<500000 ; i++);
}


int main(void){

	GPIO_PinConfig_t myGPIO;
	myGPIO.port = GPIOA;
	myGPIO.pin = 5;
	myGPIO.mode = OUTPUT_MODE;
	myGPIO.mode_type = GPIO_OUT_PUSHPULL;
	myGPIO.speed = GPIO_MODE_OUT_50MHz;

	GPIO_Init(&myGPIO, ENABLE);

	//configure_gpio_interrupt(GPIOB, 4, RISING_EDGE);
	//GPIO_IRQPriorityConfig(EXTI4_IRQn, 15);
	//GPIO_IRQInterruptConfig(EXTI4_IRQn, ENABLE);
	configure_gpio_interrupt(GPIOA, 4, RISING_EDGE);
	enable_gpio_interrupt(4, EXTI4_IRQn);

	while(1){
		//gpio_toggle(GPIOA, 5);
		delay();
	}



	return 0;
}

void EXTI4_IRQHandler(void){
	//Handle the interrupt
	//GPIO_IRQHandling(4);
	//gpio_toggle(GPIOC, 13);
	clear_gpio_interrupt(4);
	gpio_toggle(GPIOC, 13);
}

