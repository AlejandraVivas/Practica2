/*
 * I2C.c
 *
 *  Created on: 29/03/2017
 *      Author: Alejandra Meza
 */

#include "I2C.h"

i2c_master_handle_t i2c_handle;
volatile bool completionFlag = false;
volatile bool nakFlag = false;


void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData){
	/* Signal transfer success when received success status. */
	if (status == kStatus_Success)
	{
		completionFlag = true;
	}
	/* Signal transfer success when received success status. */
	if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
	{
		nakFlag = true;
	}
}

uint8_t i2c_init(void){
	i2c_master_config_t masterConfig;

	/*
	 * masterConfig.baudRate_Bps = 100000U;
	 * masterConfig.enableHighDrive = false;
	 * masterConfig.enableStopHold = false;
	 * masterConfig.glitchFilterWidth = 0U;
	 * masterConfig.enableMaster = true;
	 */

	I2C_MasterGetDefaultConfig(&masterConfig);

	masterConfig.baudRate_Bps = I2C_BAUDRATE;

	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(I2C0_CLK_SRC));
	I2C_MasterTransferCreateHandle(I2C0, &i2c_handle, i2c_master_callback, NULL);
}

bool I2C_RtcRead(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &i2c_handle, &masterXfer);
	/*  wait for transfer completed. */
	while((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool I2C_RtcWrite(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &value;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &i2c_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}



bool I2C_MemoryRead(I2C_Type *base, uint8_t device_addr, uint16_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 2;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &i2c_handle, &masterXfer);
	/*  wait for transfer completed. */
	while((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}


bool I2C_MemoryWrite(I2C_Type *base, uint8_t device_addr, uint16_t reg_addr, uint8_t *txBuff, uint32_t txSize)
{

	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 2;
	masterXfer.data = txBuff;
	masterXfer.dataSize = txSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &i2c_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}
