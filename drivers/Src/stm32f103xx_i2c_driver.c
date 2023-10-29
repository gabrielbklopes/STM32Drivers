/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Oct 11, 2023
 *      Author: gabri
 */


#include "stm32f103xx.h"
#include "stm32f103xx_i2c_driver.h"
#include <stdint.h>

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[8] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t flag);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


uint32_t RCC_GetPLLOutputClock(){

	return 0;
}



uint32_t RCC_GetPCLK1Value(){

	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->RCC_CFGR >> 2) & 0x3;

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}


	//AHB
	temp = ((RCC->RCC_CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	//APB1
	temp = ((RCC->RCC_CFGR >> 8) & 0x7);

	if(temp < 4){
		apb1p = 1;
	}else{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk/ahbp) / apb1p;

	return pclk1;
}



/*
 * Peripheral Clock setup
 */
/**
  * @Func       : I2C_PeriClockControl
  * @brief      : Enables and Disables peripheral clock for the given I2C port
  * @parameters : Base address of the GPIO peripheral
  * @parameters : ENABLE or DISABLE macros
  * @return     : none
  * @note       : none
  */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
			if(pI2Cx == I2C1){
				I2C1_PCLK_EN();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_EN();
			}
		}else{
			if(pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}
		}
}

/**
  * @Func       : I2C_PeripheralControl
  * @brief      : Enables and Disables peripheral for the given I2C port
  * @parameters : Base address of the GPIO peripheral
  * @parameters : ENABLE or DISABLE macros
  * @return     : none
  * @note       : none
  */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	//tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	//trise configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){

		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else{

		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300 ) / 1000000000U) +1 ;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}



//void I2C_DeInit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	//while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to w(0) (tota 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, 0);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	//while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until Len becomes 0

	while(Len > 0){
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)); //wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)

	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));


	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);



}

void I2C_ReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr){

	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to low)
	//while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, 1);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));


	//procedure to read only 1 byte from slave
	if(Len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait until RXNE becomes 1
		//while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		//Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	//procedure to read data from slave when Len > 1
	if(Len > 1){
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--){
			//wait until RXNE becomes 1
			while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			if(i == 2){ //if last 2 bytes are remaining
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	//pI2Cx->CR1 |= (1 << I2C_CR1_START);
	pI2Cx->CR1 |= (1 << 8);

}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t flag){

	if(flag == 0){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr &= ~(1); // SlaveAddr is Slave address + r/nw bit = 0
		pI2Cx->DR = SlaveAddr;
	}else if(flag == 1){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr |= 1;
		pI2Cx->DR = SlaveAddr;
	}

}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){

	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		//enable ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else{
		//disable ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

