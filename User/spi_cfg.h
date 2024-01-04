/******************************************************************************
* File Name:   spi_cfg.h
*
*  Created on: 2023/06/14
*      Author: Yifei.Su
*
* Description:
*   spi configuration for pi tool version 2.0
*
*******************************************************************************/

#ifndef USER_SPI_CFG_H_
#define USER_SPI_CFG_H_


/*******************************************************************************
* Include Files
*******************************************************************************/
#include "debug.h"


/*******************************************************************************
* Macro Definition
*******************************************************************************/
#define     SPI_TX_BUF_SIZE     12
#define     SPI_RX_BUF_SIZE     12


/*******************************************************************************
* External Variables
*******************************************************************************/
extern u8 spi_tx_buf[SPI_TX_BUF_SIZE];
extern u8 spi_rx_buf[SPI_RX_BUF_SIZE];

extern volatile u8 spi_dma_flag;


/*******************************************************************************
* External Functions
*******************************************************************************/
extern void spi1_full_duplex_init(void);

extern void spi_tx_dma_init(SPI_TypeDef * pSpix, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize);
extern void spi_rx_dma_init(SPI_TypeDef * pSpix, uint32_t RCC_AHBPeriph, DMA_Channel_TypeDef* DMA_CHx,
                     u32 ppadr, u32 memadr, u16 bufsize);

extern void spi_dma_interrupt_enable(DMA_Channel_TypeDef *DMAy_Channelx, uint32_t DMA_IT, uint32_t DMAy_Channelx_IRQn);

extern void spix_write_array(SPI_TypeDef * SPIx, u32 wrLen, u8 * pDataBuf);
extern void spix_read_array(SPI_TypeDef * SPIx, u32 wrLen,  u8 * wrDataBuf, u8 * rdDataBuf);


#endif /* USER_SPI_CFG_H_ */
