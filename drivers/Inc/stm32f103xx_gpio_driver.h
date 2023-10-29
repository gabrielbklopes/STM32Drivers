/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Gabriel Lopes
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * This is a configuration structure for a GPIO pin
 */
/*
typedef struct{

	  uint32_t PinNumber;       				//!< Specifies the GPIO pins to be configured.
	                           	   	   	   	   //This parameter can be any value of @ref GPIO_pins_define

	  uint32_t PinMode;      				   //!< Specifies the operating mode for the selected pins.
	                           	   	   	   	  //This parameter can be a value of @ref GPIO_mode_define

	  uint32_t GPIO_PinPuPdControl;      	  //!< Specifies the Pull-up or Pull-Down activation for the selected pins.
	                           	   	   	 	 //This parameter can be a value of @ref GPIO_pull_define

	  uint32_t GPIOSpeed;     				//!< Specifies the speed for the selected pins.
	                           	   	   	   //This parameter can be a value of @ref GPIO_speed_define
}GPIO_PinConfig_t;
*/

typedef struct{
	GPIO_RegDef_t *port;
	uint32_t pin;
	uint32_t mode;
	uint32_t mode_type;
	uint32_t pull;
	uint32_t speed;
	uint32_t alt_func;
}GPIO_PinConfig_t;


typedef enum{
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
}edge_select;

/*
 * This is a Handler structure for a GPIO pin
 */

/*typedef struct{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; //this holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // This holds GPIO pin configuration settings

}GPIO_Handler_t;*/


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0 				0
#define GPIO_PIN_NO_1 				1
#define GPIO_PIN_NO_2 				2
#define GPIO_PIN_NO_3 				3
#define GPIO_PIN_NO_4 				4
#define GPIO_PIN_NO_5 				5
#define GPIO_PIN_NO_6 				6
#define GPIO_PIN_NO_7 				7
#define GPIO_PIN_NO_8 				8
#define GPIO_PIN_NO_9 				9
#define GPIO_PIN_NO_10 				10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12 				12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

//PIN MODE
#define OUTPUT_MODE			((uint32_t) 0x01)
#define INPUT_MODE			((uint32_t) 0x02)

//Define the pin mode
#define GPIO_MODE_IN     			((uint32_t)0x00)
#define GPIO_MODE_OUT_10MHz    		((uint32_t)0x01)
#define GPIO_MODE_OUT_2MHz  		((uint32_t)0x02)
#define GPIO_MODE_OUT_50MHz 		((uint32_t)0x03)

//if pin mode is defined as input
#define GPIO_ANALOG_MODE 			((uint32_t)0x00)
#define GPIO_FLOATING_IN 			((uint32_t)0x01)
#define GPIO_IN_PUPD 				((uint32_t)0x02)
#define GPIO_RESERVED 				((uint32_t)0x03)

//if pin mode is defined as output
#define GPIO_OUT_PUSHPULL 			((uint32_t)0x00)
#define GPIO_OUT_OPENDRAIN 			((uint32_t)0x01)
#define	GPIO_ALTFN_OUT_PUSHPULL 	((uint32_t)0x02)
#define GPIO_ALTFN_OUT_OPENDRAIN 	((uint32_t)0x03)

//HIGH BIT POSITIONS FOR CRH REGISTER CNFYG AND MODE
#define CNF_POS_BIT1				(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2				(PINPOS[pinNumber] + 3)


/**********************************************************************************
 *   						API´s supported by this driver
 *         For more information about the API's the function definitions
 **********************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO CONFIGURATION FUNCTIONS
 */

void GPIO_Init(GPIO_PinConfig_t *pGPIOx, uint8_t EnorDi);
void config_pin (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t mode_type);
void config_pin_speed (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode);

//**************************************************************************
//                         GPIO PIN FUNCTIONS
void gpio_write(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint8_t state);
uint8_t gpio_read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void gpio_toggle(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber);

//**************************************************************************
//                         INTERRUPT FUNCTIONS

void configure_gpio_interrupt(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pinNumber);
//void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
//void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
//void GPIO_IRQHandling(uint8_t pinNumber);

//void GPIO_Init(GPIO_Handler_t *pGPIOHandler);
//void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and Write
 */
//uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
//uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
//void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
//void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
//void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
//void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
//void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
