/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: 13 de out de 2023
 *      Author: gabri
 */


#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"


//extern void initialise_monitor_handles();

#define MY_ADDR 0x61
#define SLAVE_ADDR (0x68)
I2C_Handle_t I2C1Handle;
uint8_t some_data[] = {0xFE};
uint8_t rcv_buffer[1];//[32];

void I2C1_GPIOInits(){
	GPIO_PinConfig_t I2CPins;
	I2CPins.port = GPIOB;
	I2CPins.mode = OUTPUT_MODE;
	I2CPins.mode_type = GPIO_ALTFN_OUT_OPENDRAIN;
	I2CPins.speed = GPIO_MODE_OUT_50MHz;

	//scl
	I2CPins.pin = 6;
	GPIO_Init(&I2CPins, ENABLE);

	//sda
	I2CPins.pin = 7;
	GPIO_Init(&I2CPins, ENABLE);
}

void I2C1_Inits(){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);


}

void delay(void){
	for(uint32_t i=0; i<1000000 ; i++);
}

void GPIO_ButtonInit(void){

	GPIO_PinConfig_t GPIOBtn;
	GPIOBtn.port = GPIOA;
	GPIOBtn.pin = 4;
	GPIOBtn.mode = OUTPUT_MODE;
	GPIOBtn.mode_type = GPIO_OUT_PUSHPULL;
	GPIOBtn.speed = GPIO_MODE_OUT_50MHz;

	GPIO_Init(&GPIOBtn, ENABLE);

	GPIO_PinConfig_t myGPIO;
	myGPIO.port = GPIOA;
	myGPIO.pin = 5;
	myGPIO.mode = OUTPUT_MODE;
	myGPIO.mode_type = GPIO_OUT_PUSHPULL;
	myGPIO.speed = GPIO_MODE_OUT_50MHz;

	GPIO_Init(&myGPIO, ENABLE);
}


int main(void){


	uint8_t commandcode;
	uint8_t len = 0;

	//initialise_monitor_handles();

	//printf("Application is running\n");

	GPIO_ButtonInit();

	I2C1_GPIOInits();

	I2C1_Inits();

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);



	while(1){
		//wait until button is pressed
		//while( !gpio_read(GPIOA, 0) );

		//while(1){
		//	gpio_toggle(GPIOC, 13);
		//	for(int i=0; i<=5000000; i++);
		//}

		delay();
		//I2C_MasterSendData(&I2C1Handle, some_data, 1, SLAVE_ADDR);

		commandcode = 0x51;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);
		I2C_ReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR);

		if(len == 0){
			gpio_toggle(GPIOA, 5);
			delay();
		}else{
			gpio_toggle(GPIOA, 4);
			delay();
		}

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);
		//I2C_ReceiveData(&I2C1Handle, rcv_buffer, len, SLAVE_ADDR);

		/*if(rcv_buffer[1] == 0xAF){
			gpio_toggle(GPIOA, 5);
			for(int i=0; i<=5000000; i++);
		}else{
			gpio_toggle(GPIOA, 4);
			for(int i=0; i<=5000000; i++);
		}*/

		//rcv_buffer[len+1] = '\0';

		//printf("Data: %s", rcv_buffer);

		//send some data to slave
		//I2C_MasterSendData(&I2C1Handle, some_data, 1, SLAVE_ADDR);
	}

	/*GPIO_PinConfig_t myGPIO;
	myGPIO.port = GPIOC;
	myGPIO.pin = 13;
	myGPIO.mode = OUTPUT_MODE;
	myGPIO.mode_type = GPIO_OUT_PUSHPULL;
	myGPIO.speed = GPIO_MODE_OUT_50MHz;

	GPIO_Init(&myGPIO, ENABLE);

	configure_gpio_interrupt(GPIOB, 4, RISING_EDGE);
	GPIO_IRQPriorityConfig(EXTI4_IRQn, 15);
	GPIO_IRQInterruptConfig(EXTI4_IRQn, ENABLE);


	while(1){
	//	gpio_toggle(GPIOC, 13);
	//	for(int i=0; i<=5000000; i++);
	}*/



	return 0;
}


