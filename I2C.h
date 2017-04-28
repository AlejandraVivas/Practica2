/*
 * I2C.h
 *
 *  Created on: 29/03/2017
 *      Author: Alejandra Meza
 */

#ifndef SOURCE_I2C_H_
#define SOURCE_I2C_H_

#include "fsl_i2c.h"

#define I2C_BAUDRATE 100000U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
uint8_t i2c_init(void);
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);

bool I2C_RtcRead(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
bool I2C_RtcWrite(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

bool I2C_MemoryRead(I2C_Type *base, uint8_t device_addr, uint16_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
bool I2C_MemoryWrite(I2C_Type *base, uint8_t device_addr, uint16_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);



#endif /* SOURCE_I2C_H_ */
