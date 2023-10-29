/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Gabriel Lopes
 */

#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx.h"
#include <stdint.h>

uint32_t PINPOS[16] = {
		(0x00), //pin0
		(0x04), //pin1
		(0x08), //pin2
		(0x0C), //pin3
		(0x10), //pin4
		(0x14), //pin5
		(0x18), //pin6
		(0x1C), //pin7
		(0x00), //pin8
		(0x04), //pin9
		(0x08), //pin10
		(0x0C), //pin11
		(0x10), //pin12
		(0x14), //pin13
		(0x18), //pin14
		(0x1C)  //pin15
};


/*
 * Peripheral Clock setup
 */
/**
  * @Func       : GPIO_PeriClockControl
  * @brief      : Enables and Disables peripheral clock for the given GPIO port
  * @parameters : Base address of the GPIO peripheral
  * @parameters : ENABLE or DISABLE macros
  * @return     : none
  * @note       : none
  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)	{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
	}
}

/*
 * Init and DeInit
 */
/**
  * @Func       : GPIO_Init
  * @brief      : Initial configuration for the given GPIO port
  * @parameters : Base address of the GPIO peripheral
  * @return     : none
  * @note       : none
  */

void GPIO_Init(GPIO_PinConfig_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx->port == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx->port == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx->port == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx->port == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx->port == GPIOE){
			GPIOE_PCLK_EN();
		}
	}else{
		if(pGPIOx->port == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx->port == GPIOB)	{
			GPIOB_PCLK_DI();
		}else if(pGPIOx->port == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx->port == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx->port == GPIOE){
			GPIOE_PCLK_DI();
		}
	}

	config_pin(pGPIOx->port, pGPIOx->pin, pGPIOx->mode_type);
	config_pin_speed(pGPIOx->port, pGPIOx->pin, pGPIOx->speed, pGPIOx->mode);
}

void config_pin (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t mode_type){
	if(pinNumber >= 8){ // CONTROL HIGH REGISTER
		switch(mode_type){
			case GPIO_OUT_PUSHPULL | GPIO_ANALOG_MODE:
				pGPIOx->GPIO_CRH &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
				break;
			case GPIO_OUT_OPENDRAIN | GPIO_FLOATING_IN:
				pGPIOx->GPIO_CRH &= ~( 1 << CNF_POS_BIT2);
				pGPIOx->GPIO_CRH |= (1 << CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_PUSHPULL | GPIO_IN_PUPD:
				pGPIOx->GPIO_CRH |= GPIO_ALTFN_OUT_PUSHPULL << (CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_OPENDRAIN:
					pGPIOx->GPIO_CRH |= GPIO_ALTFN_OUT_OPENDRAIN << (CNF_POS_BIT1);
				break;
		}
	}else{ // CONTROL LOW REGISTER
		switch(mode_type){
			case GPIO_OUT_PUSHPULL | GPIO_ANALOG_MODE:
				pGPIOx->GPIO_CRL &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
				break;
			case GPIO_OUT_OPENDRAIN | GPIO_FLOATING_IN:
				pGPIOx->GPIO_CRL &= ~( 1 << CNF_POS_BIT2);
				pGPIOx->GPIO_CRL |= (1 << CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_PUSHPULL | GPIO_IN_PUPD:
				pGPIOx->GPIO_CRL |= GPIO_ALTFN_OUT_PUSHPULL << (CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_OPENDRAIN:
				pGPIOx->GPIO_CRL |= GPIO_ALTFN_OUT_OPENDRAIN << (CNF_POS_BIT1);
				break;
		}
	}
}

void config_pin_speed (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode){

	if(pinNumber >= 8){
		if(mode == INPUT_MODE)
			pGPIOx->GPIO_CRH &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
		else
			pGPIOx->GPIO_CRH = (pinSpeed << (PINPOS[pinNumber]));
	}else{
		if(mode == INPUT_MODE)
			pGPIOx->GPIO_CRL &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
		else
			pGPIOx->GPIO_CRL |= (pinSpeed << (PINPOS[pinNumber]));
	}
}

void gpio_write(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint8_t state){

	if(state){
		pGPIOx->GPIO_BSRR = (1 << pinNumber);
	}else{
		pGPIOx->GPIO_BSRR = (1 << (pinNumber +16));
	}

}


uint8_t gpio_read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->GPIO_IDR >> PinNumber) & (0x00000001));
	return value;
}



void gpio_toggle(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber){

	pGPIOx->GPIO_ODR ^=(1<<pinNumber);

}

void configure_gpio_interrupt(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, edge_select edge){
	GPIO_PCLK_EN_ALT_FUNC();

	if(pGPIOx == GPIOA){
		switch(pinNumber)
		{
			case 0:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
				break;
			case 1:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;
				break;
			case 2:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
				break;
			case 3:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
				break;
			case 4:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
				break;
			case 5:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
				break;
			case 6:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
				break;
			case 7:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
				break;
			case 8:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
				break;
			case 9:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
				break;
			case 10:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
				break;
			case 11:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
				break;
			case 12:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
				break;
			case 13:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
				break;
			case 14:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
				break;
			case 15:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOB){
		switch(pinNumber)
		{
			case 0:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
				break;
			case 1:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
				break;
			case 2:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
				break;
			case 3:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
				break;
			case 4:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
				break;
			case 5:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
				break;
			case 6:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
				break;
			case 7:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
				break;
			case 8:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
				break;
			case 9:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
				break;
			case 10:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
				break;
			case 11:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
				break;
			case 12:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
				break;
			case 13:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
				break;
			case 14:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
				break;
			case 15:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOC){
		switch(pinNumber)
		{
			case 0:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
				break;
			case 1:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
				break;
			case 2:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
				break;
			case 3:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
				break;
			case 4:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
				break;
			case 5:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
				break;
			case 6:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
				break;
			case 7:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;
				break;
			case 8:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
				break;
			case 9:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
				break;
			case 10:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
				break;
			case 11:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
				break;
			case 12:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
				break;
			case 13:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
				break;
			case 14:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
				break;
			case 15:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOD){
		switch(pinNumber)
		{
			case 0:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
				break;
			case 1:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
				break;
			case 2:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
				break;
			case 3:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
				break;
			case 4:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
				break;
			case 5:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
				break;
			case 6:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
				break;
			case 7:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
				break;
			case 8:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
				break;
			case 9:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
				break;
			case 10:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
				break;
			case 11:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
				break;
			case 12:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
				break;
			case 13:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
				break;
			case 14:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
				break;
			case 15:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
				break;
			default:
				break;
		}


	}

	if(pGPIOx == GPIOE){
		switch(pinNumber)
		{
			case 0:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI0_PE;
				break;
			case 1:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI1_PE;
				break;
			case 2:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI2_PE;
				break;
			case 3:
				AFIO->AFIO_EXTICR[0] = AFIO_EXTICR1_EXTI3_PE;
				break;
			case 4:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI4_PE;
				break;
			case 5:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI5_PE;
				break;
			case 6:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI6_PE;
				break;
			case 7:
				AFIO->AFIO_EXTICR[1] = AFIO_EXTICR2_EXTI7_PE;
				break;
			case 8:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI8_PE;
				break;
			case 9:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI9_PE;
				break;
			case 10:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI10_PE;
				break;
			case 11:
				AFIO->AFIO_EXTICR[2] = AFIO_EXTICR3_EXTI11_PE;
				break;
			case 12:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI12_PE;
				break;
			case 13:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI13_PE;
				break;
			case 14:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI14_PE;
				break;
			case 15:
				AFIO->AFIO_EXTICR[3] = AFIO_EXTICR4_EXTI15_PE;
				break;
			default:
				break;
		}
	}


	if(edge == RISING_EDGE)
		EXTI->EXTI_RTSR |= 1 << pinNumber;
	if(edge == FALLING_EDGE)
		EXTI->EXTI_FTSR |= 1 << pinNumber;
	if(edge == RISING_FALLING_EDGE){
		EXTI->EXTI_RTSR |= 1 << pinNumber;
		EXTI->EXTI_FTSR |= 1 << pinNumber;
	}




}

void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber){

	//enable interrupt in EXTI
	EXTI->EXTI_IMR |= 1 << pinNumber;
	//enable interrupt in NVIC
	NVIC_EnableIRQ(irqNumber);
}

void clear_gpio_interrupt(uint32_t pinNumber){

	EXTI->EXTI_PR |= (1 << pinNumber);
}

/*void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){

			//program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){

			//program ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96){

			//program ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	}else{
		if(IRQNumber <= 31){

			//program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){

			//program ISER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96){

			//program ISER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}
}*/


/*void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}*/

/*void GPIO_IRQHandling(uint8_t pinNumber){

	if(EXTI->EXTI_PR & (1 << pinNumber)){
		//clear
		EXTI->EXTI_PR |= (1 << pinNumber);
	}
}*/




/*void GPIO_Init(GPIO_Handler_t *pGPIOHandler){
	//1. configure the mode of the GPIO pin
	uint32_t temp = 0;
	if(pGPIOHandler->GPIO_PinConfig.PinMode <= 3){
		if(pGPIOHandler->GPIO_PinConfig.PinNumber > 7){
			// Non interrupt mode
			//temp = (pGPIOHandler->GPIO_PinConfig.PinMode << (4 *  pGPIOHandler->GPIO_PinConfig.PinNumber));
			//pGPIOHandler->pGPIOx->GPIO_CRH &=~(0x3 << pGPIOHandler->GPIO_PinConfig.PinNumber);
			//pGPIOHandler->pGPIOx->GPIO_CRH |= temp;
			switch (pGPIOHandler->GPIO_PinConfig.PinMode)
			{
				case GPIO_MODE_IN:

					break;
				case (GPIO_MODE_OUT_10MHz || GPIO_MODE_OUT_2MHz || GPIO_MODE_OUT_50MHz ):
					pGPIOHandler->pGPIOx->GPIO_CRH |=  (1 << ((pGPIOHandler->GPIO_PinConfig.PinNumber - 8)*4)) | (1 << (((pGPIOHandler->GPIO_PinConfig.PinNumber - 8)*4)+1));
					pGPIOHandler->pGPIOx->GPIO_CRH &= ~((1<<22) | (1<<23));
					break;

				default:
					break;
			}

		}else{
			// Non interrupt mode
			temp = (pGPIOHandler->GPIO_PinConfig.PinMode << (4 *  pGPIOHandler->GPIO_PinConfig.PinNumber));
			pGPIOHandler->pGPIOx->GPIO_CRH &=~(0x3 << pGPIOHandler->GPIO_PinConfig.PinNumber);
			pGPIOHandler->pGPIOx->GPIO_CRH |= temp;
		}


	}else{
		//interrupt mode
	}

	temp = 0;

	//2. configure the CNF
	// this configuration will work based on bellow list

	 * In input mode:
	 * 00: Analog mode; 01: Floating input (reset state); 10: Input with pull-up/pull-down; 11: Reserved
	 * In output mode:
	 * 00: General purpose output push-pull; 01: General purpose output Open-drain: 10: Alternate function output Push-pull; 11: Alternate function output Open-Drain

	//temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdControl << (4*pGPIOHandler->GPIO_PinConfig.PinNumber + 2));
	//pGPIOHandler->pGPIOx->GPIO_CRL &=~(0x3 << (pGPIOHandler->GPIO_PinConfig.PinNumber+2) );
	//pGPIOHandler->pGPIOx->GPIO_CRL |= temp;

	//temp = 0;



	//5. configure the alt functionality
	//if((pGPIOHandler->GPIO_PinConfig.PinMode == GPIO_ALTFN_OUT_PUSHPULL) || (pGPIOHandler->GPIO_PinConfig.PinMode == GPIO_ALTFN_OUT_OPENDRAIN)){
		// TODO
	//}


}
*/

/**
  * @Func       : GPIO_DeInit
  * @brief      : Disconfiguration for the given GPIO port
  * @parameters : Base address of the GPIO peripheral
  * @return     : none
  * @note       : none
  */
/*void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
		}else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}

}*/

/*
 * Data read and Write
 */
/**
  * @Func       : GPIO_ReadFromInputPin
  * @brief      : Read data from defined pin
  * @parameters : Base address of the GPIO peripheral
  * @parameters : Pin Number
  * @return     : 8 bits value from the read pin
  * @note       : none
  */

/**
  * @Func       : GPIO_ReadFromInputPort
  * @brief      : Read data from defined port
  * @parameters : Base address of the GPIO peripheral
  * @return     : 16 bits value from the read port
  * @note       : none
  */
/*uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->GPIO_IDR;
	return value;
}*/

/**
  * @Func       : GPIO_WriteToOutputPin
  * @brief      : Write data to defined pin
  * @parameters : Base address of the GPIO peripheral
  * @parameters : Pin Number
  * @parameters : Value to be written
  * @return     : none
  * @note       : none
  */
/*void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx->GPIO_ODR |= (1 << PinNumber);
	}else{
		pGPIOx->GPIO_ODR &= ~(1 << PinNumber);
	}
}*/

/**
  * @Func       : GPIO_WriteToOutputPort
  * @brief      : Write data to defined port
  * @parameters : Base address of the GPIO peripheral
  * @parameters : Value to be written
  * @return     : none
  * @note       : none
  */
/*void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->GPIO_ODR = Value;
}*/

/**
  * @Func       : GPIO_ToggleOutputPin
  * @brief      : Toggle a defined pin
  * @parameters : Base address of the GPIO peripheral
  * @parameters : Pin Number
  * @return     : none
  * @note       : none
  */
/*void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	//pGPIOx->GPIO_ODR ^= (1 << PinNumber);
	pGPIOx->GPIO_BSRR = (1 << 13);
}*/

/*
 * IRQ Configuration and ISR Handling
 */
/**
  * @Func       : GPIO_IRQConfig
  * @brief      : none
  * @parameters : none
  * @parameters : none
  * @return     : none
  * @note       : none
  */
/*void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}*/

/**
  * @Func       : GPIO_IRQHandling
  * @brief      : none
  * @parameters : none
  * @parameters : none
  * @return     : none
  * @note       : none
  */
/*void GPIO_IRQHandling(uint8_t PinNumber){

}*/
