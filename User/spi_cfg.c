/******************************************************************************
* File Name:   spi_cfg.c
*
*  Created on: 2023/06/27
*      Author: Yifei.Su
*
* Description:
*   spi peripheral configuration which includes corresponding dma operation
*
*******************************************************************************
* Copyright (c) 2021 Pisemi Semiconductor Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by xxxxxx.
*******************************************************************************/


#include "spi_cfg.h"


/*******************************************************************************
* Global Variables
*******************************************************************************/
u8 spi_tx_buf[SPI_TX_BUF_SIZE];
u8 spi_rx_buf[SPI_RX_BUF_SIZE];

volatile u8 spi_dma_flag = 0;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void spi1_full_duplex_init(void);

void spi_tx_dma_init(SPI_TypeDef * SPIx, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize);
void spi_rx_dma_init(SPI_TypeDef * SPIx, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize);

void spi_dma_interrupt_enable(DMA_Channel_TypeDef *DMAy_Channelx, uint32_t DMA_IT, uint32_t DMAy_Channelx_IRQn);

static u8 spi_read_write_byte(SPI_TypeDef * SPIx, u8 TxData);
void spix_write_array(SPI_TypeDef * SPIx, u32 wrLen, u8 * pDataBuf);
void spix_read_array(SPI_TypeDef * SPIx, u32 wrLen,  u8 * wrDataBuf, u8 * rdDataBuf);


void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/*********************************************************************
 * @fn      spi1_full_duplex_init
 *
 * @brief   Initializes the SPI1 peripheral.
 *
 * @param   none
 *
 * @return  none
 *
 * @notice  every spi pins can map with different gpio port
 *          so use different spix_init for each spi instance and corresponding gpio pins
 *          this function master: SPI1_SCK(PA5)\SPI1_MISO(PA6)\SPI1_MOSI(PA7)\CS(PA4).
 */
void spi1_full_duplex_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure = {0};

    /* enable spi and corresponding gpio clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    /* configure spi corresponding gpio pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);                            // CS

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      // SCLK

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      // MISO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      //MOSI

    /* configure spi */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      // spi mode0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;    // spi mode0
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;     // spi rate is 769KHz
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;     // spi rate is 6MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* enable spi */
    SPI_Cmd(SPI1, ENABLE);
}


/*********************************************************************
 * @fn      spi_tx_dma_init
 *
 * @brief   Initializes the DMAy Channelx tx(mem -> per) configuration.
 *
 * @param   SPIx - x can be 1, 2 or 3 to select the SPI peripheral.
 *          RCC_AHBPeriph - specifies the AHB peripheral to gates its clock.
 *            RCC_AHBPeriph_DMA1.
 *            RCC_AHBPeriph_DMA2.
 *          DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 *
 * @notice  this function configures DMA as memory -> peripheral mode
 */
void spi_tx_dma_init(SPI_TypeDef * SPIx, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure={0};

    /* enable DMA clock */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph, ENABLE );

    /* configure DMA */
    DMA_DeInit(DMA_CHx);    // should disable DMA before configuration

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

    /* enable connection between spi and DMA channel */
    SPI_I2S_DMACmd( SPIx, SPI_I2S_DMAReq_Tx, ENABLE );
}


/*********************************************************************
 * @fn      spi_rx_dma_init
 *
 * @brief   Initializes the DMAy Channelx rx(per -> mem) configuration.
 *
 * @param   SPIx - x can be 1, 2 or 3 to select the SPI peripheral.
 *          RCC_AHBPeriph - specifies the AHB peripheral to gates its clock.
 *            RCC_AHBPeriph_DMA1.
 *            RCC_AHBPeriph_DMA2.
 *          DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void spi_rx_dma_init(SPI_TypeDef * SPIx, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure={0};

    /* enable DMA clock */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph, ENABLE );

    /* configure DMA */
    DMA_DeInit(DMA_CHx);    // should disable DMA before configuration

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

    /* enable connection between spi and DMA channel */
    SPI_I2S_DMACmd( SPIx, SPI_I2S_DMAReq_Rx, ENABLE );
}


/*********************************************************************
 * @fn      spi_dma_interrupt_enable
 *
 * @brief   Enable specified DMA channel with corresponding interrupt type
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *          1 to 7 for DMA1 and 1 to 11 for DMA2 to select the DMA Channel.
 *
 *          DMA_IT - specifies the DMA interrupts sources to be enabled.
 *           DMA_IT_TC - Transfer complete interrupt mask
 *           DMA_IT_HT - Half transfer interrupt mask
 *           DMA_IT_TE -  Transfer error interrupt mask
 *
 *          DMAy_Channelx_IRQn - DMAy Channelx corresponding IRQ number in vector table
 *
 * @return  none
 */
void spi_dma_interrupt_enable(DMA_Channel_TypeDef *DMAy_Channelx, uint32_t DMA_IT, uint32_t DMAy_Channelx_IRQn)
{
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    /* enable DMAy Channelx interrupt */
    DMA_ITConfig(DMAy_Channelx, DMA_IT, ENABLE);

    /* enable DMA IRQ in NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = DMAy_Channelx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*********************************************************************
 * @fn      spi_read_write_byte
 *
 * @brief   SPIx read or write one byte.
 *
 * @param   SPIx - SPI peripheral instance.
 *          pTxData - write one byte data.
 *
 * @return  Read one byte data.
 */
static u8 spi_read_write_byte(SPI_TypeDef * SPIx, u8 TxData)
{
    u8 i = 0;

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }

    SPI_I2S_SendData(SPIx, TxData);
    i = 0;

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }

    return SPI_I2S_ReceiveData(SPIx);
}


/*********************************************************************
 * @fn      spix_write_array
 *
 * @brief   SPIx send data to slave from specified data buffer
 *
 * @param   SPIx - x can be 1, 2 or 3 to select the SPI peripheral.
 *          wrLen - write data number.
 *          pDataBuf - pointer of write data buffer.
 *
 * @return  none.
 *
 * @note    the cs control is outside this function
 */
void spix_write_array(SPI_TypeDef * SPIx, u32 wrLen, u8 * pDataBuf)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);

    for(u32 cnt=0; cnt<wrLen; cnt++)
    {
        spi_read_write_byte(SPIx, *(pDataBuf + cnt));
    }

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}


/*********************************************************************
 * @fn      spix_read_array
 *
 * @brief   SPIx read data from slave and store data into specified buffer
 *
 * @param   SPIx - x can be 1, 2 or 3 to select the SPI peripheral.
 *          wrLen - read data number.
 *          wrDataBuf - pointer of write data buffer.
 *          rdDataBuf - pointer of read data buffer.
 *
 * @return  none.
 *
 * @note    the cs control is outside this function
 */
void spix_read_array(SPI_TypeDef * SPIx, u32 wrLen,  u8 * wrDataBuf, u8 * rdDataBuf)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);

    for(u32 cnt=0; cnt<wrLen; cnt++)
    {
        *(rdDataBuf + cnt) = spi_read_write_byte(SPIx, *(wrDataBuf + cnt));
    }

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}


/*********************************************************************
 * @fn      DMA1_Channel2_IRQHandler
 *
 * @brief   DMA1_Channel2_IRQ Handler
 *
 * @param   none
 *
 * @return  none
 */
void DMA1_Channel2_IRQHandler(void)
{
    printf("current dma counter number is %d\r\n", DMA_GetCurrDataCounter(DMA1_Channel2));
    printf("enter DMA SPI RX irq\r\n");

    spi_dma_flag = 1;

    DMA_ClearFlag(DMA1_FLAG_HT2);   // clear half transmission interrupt flag
//    DMA_ClearFlag(DMA1_FLAG_TC2);   // clear complete transmission interrupt flag

}

