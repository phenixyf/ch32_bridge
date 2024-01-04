/*
 * i2c_cfg.h
 *
 *  Created on: Aug 30, 2023
 *      Author: pheni
 */

#ifndef USER_I2C_CFG_H_
#define USER_I2C_CFG_H_


/*******************************************************************************
* Include Files
*******************************************************************************/
#include "debug.h"


/*******************************************************************************
* Macro Definition
*******************************************************************************/
#define     I2C_BOUND               10000
#define     I2C_OWN_ADDRESS         0x00
#define     I2C_SLAVE_ADDRESS       0xE0
#define     I2C_BUF_SIZE            64
#define     I2C_POLLING_TIMEOUT     10000   // 5000 means 1.67ms


/*******************************************************************************
* External Variables
*******************************************************************************/
extern u8 i2c_tx_buf[I2C_BUF_SIZE];
extern u8 i2c_rx_buf[I2C_BUF_SIZE];


/*******************************************************************************
* External Functions
*******************************************************************************/
extern void i2c1_master_init(u32 bound, u16 address);
extern uint8_t i2c1_write_array(u8 slaveAddr, const u8 *pBuffer, u16 dataNum);
extern uint8_t i2c1_read_array(u8 slaveAddr, u8 *pBuffer, u16 dataNum);


#endif /* USER_I2C_CFG_H_ */
