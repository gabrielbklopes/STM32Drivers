/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: Oct 11, 2023
 *      Author: gabri
 */

#ifndef INC_STM32F103XX_I2C_DRIVER_H_
#define INC_STM32F103XX_I2C_DRIVER_H_

#include "stm32f103xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 		100000
#define I2C_SCL_SPEED_FM4k 		400000
#define I2C_SCL_SPEED_FM2K 		200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/*
 * I2C related status flags definition
 */
#define I2C_TXE_FLAG			( 1 << I2C_SR1_TxE)
#define I2C_RXNE_FLAG			( 1 << I2C_SR1_RxNE)
#define I2C_SB_FLAG				( 1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG			( 1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG			( 1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG			( 1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG			( 1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG			( 1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG				( 1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			( 1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG		( 1 << I2C_SR1_TIMEOUT)



/****************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 ****************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_ReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

uint8_t I2C_SendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */
