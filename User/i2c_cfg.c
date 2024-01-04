/*
 * i2c_cfg.c
 *
 *  Created on: Aug 30, 2023
 *      Author: pheni
 */

#include "i2c_cfg.h"


#define Address_Lenth    Address_8bit


/*******************************************************************************
* Global Variables
*******************************************************************************/
u8 i2c_tx_buf[I2C_BUF_SIZE] = {0};
u8 i2c_rx_buf[I2C_BUF_SIZE] = {0};


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void i2c1_master_init(u32 bound, u16 address);
uint8_t i2c1_write_array(u8 slaveAddr, const u8 *pBuffer, u16 dataNum);
uint8_t i2c1_read_array(u8 slaveAddr, u8 *pBuffer, u16 dataNum);


/*********************************************************************
 * @fn      i2c_master_init
 *
 * @brief   Initializes the I2C peripheral work as master mode.
 *
 * @param   none
 *
 * @return  none
 *
 * @notice  SCK(PB8)\SDA(PB9)
 */
void i2c1_master_init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitTSturcture = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
}


/*********************************************************************
 * @fn      i2c1_write_array
 *
 * @brief   Write multiple data to slave.
 *
 * @param   slaveAddr - slave address
 *          pBuffer - pointer of write data buffer.
 *          dataNum - Data number.
 *
 * @return  none
 */
uint8_t i2c1_write_array(u8 slaveAddr, const u8 *pBuffer, u16 dataNum)
{
    uint16_t to_val = 0;

    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET )
    {
       to_val++;
       if(to_val > I2C_POLLING_TIMEOUT)
           return 0;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    to_val = 0;
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
    {
       to_val++;
       if(to_val > I2C_POLLING_TIMEOUT)
           return 0;
    }

    I2C_Send7bitAddress( I2C1, slaveAddr, I2C_Direction_Transmitter );
    to_val = 0;
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
    {
       to_val++;
       if(to_val > I2C_POLLING_TIMEOUT)
           return 0;
    }

    for(int i=0; i< dataNum; i++ )
    {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) != RESET)
        {
            I2C_SendData( I2C1, *(pBuffer + i) );
            to_val = 0;
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
            {
               to_val++;
               if(to_val > I2C_POLLING_TIMEOUT)
                   return 0;
            }
        }
    }

        I2C_GenerateSTOP( I2C1, ENABLE );

    return 1;
}


/*********************************************************************
 * @fn      i2c1_read_array
 *
 * @brief   Read multiple data from slave.
 *
 * @param   slaveAddr - slave address
 *          pBuffer - pointer of read data buffer.
 *          dataNum - Data number.
 *
 * @return  temp - Read data.
 */
uint8_t i2c1_read_array(u8 slaveAddr, u8 *pBuffer, u16 dataNum)
{
    uint16_t to_val = 0;

    I2C_GenerateSTART(I2C1, ENABLE);

    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
       to_val++;
       if(to_val > I2C_POLLING_TIMEOUT)
           return 0;
    }
    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);

    to_val = 0;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
       to_val++;
       if(to_val > I2C_POLLING_TIMEOUT)
           return 0;
    }

   for(int i=0; i<dataNum; i++)
   {
       if(i == (dataNum-1))
           I2C_AcknowledgeConfig(I2C1, DISABLE);

       to_val = 0;
       while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
       {
          to_val++;
          if(to_val > I2C_POLLING_TIMEOUT)
              return 0;
       }

       *(pBuffer + i) = I2C_ReceiveData(I2C1);
   }

   I2C_GenerateSTOP(I2C1, ENABLE);

   I2C_AcknowledgeConfig(I2C1, ENABLE);

   return 1;
}


/*********************************************************************
 * @fn      i2c_master_write_array
 *
 * @brief   master send array data to slave
 *
 * @param   slaveAddr - slave address
 *          pData - pointer to data array
 *          lenData - data length
 *
 * @return  TRUE: transmit successfully
 *          FALSE: transmit fail
 *
 * @notice  this function uses polling mode to transmit data
 */
u32 i2c_master_write_array(u8 slaveAddr, u8* pData, u32 lenData, u32 timeout)
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

    I2C_GenerateSTART(I2C1, ENABLE);

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress( I2C1, 0x80, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

   for(int i=0; i< 6;i++ )
    {
       while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
//        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) != RESET)
//        {
            I2C_SendData( I2C1, *(pData+i) );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
//        }
//        printf("enter for\r\n");
    }

//    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );
    Delay_Ms(1000);

    printf("transmit stop\r\n");

    u32 status = 0;

//    /* check i2c bus is free */
//    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
////    if(I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET)
////    {
////        return SET;
////    }
//
//    /* send start */
//    I2C_GenerateSTART(I2C1, ENABLE);
//    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
//
////    do
////    {
////        status = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT );
////        status |= time_out(timeout);
////    }
////    while(0 == status);
////
////    if( (status & TIMEOUT_DONE) != 0 )
////    {
////        return TIMEOUT_DONE;
////    }
////    else {
////        status = 0;
////    }
//
//
//    /* send slave address */
//    I2C_Send7bitAddress( I2C1, 0xE0, I2C_Direction_Transmitter );
//    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );
//
////    do
////    {
////        status = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
////        status |= time_out(timeout);
////    }
////    while(0 == status);
////
////    if( (status & TIMEOUT_DONE) != 0 )
////    {
////        return TIMEOUT_DONE;
////    }
////    else {
////        status = 0;
////    }
//
//
//    /* send data */
//    for( int i=0; i< lenData; i++ )
//    {
//        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) != RESET)
//        {
//            Delay_Us(100);
//            I2C_SendData( I2C1, *(pData+i) );
//        }
//    }
//
//
////    do
////    {
////        status = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED );
////        status |= time_out(timeout);
////    }
////    while(0 == status);
////
////    if( (status & TIMEOUT_DONE) != 0 )
////    {
////        return TIMEOUT_DONE;
////    }
////    else {
////        status = 0;
////    }
//
//
//    /* send stop */
//    I2C_GenerateSTOP( I2C1, ENABLE );

    return status;
}

