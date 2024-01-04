/*
 * hid_device.c
 *
 *  Created on: Sep 28, 2023
 *      Author: pheni
 */


#include "hid_device.h"
#include "ch32v30x_usbhs_device.h"
#include "ch32v30x_usb.h"


__attribute__ ((aligned(4))) uint8_t  HID_Report_Buffer[64];              // HID Report Buffer
volatile uint8_t HID_Set_Report_Flag = SET_REPORT_DEAL_OVER;              // HID SetReport flag

volatile uint16_t Data_Pack_Max_Len = 0;                                   // UART data packet length in hid packet
volatile uint16_t Head_Pack_Len = 0;                                       // UART head packet( valid data length ) length in hid packet


uint8_t hid_tx_buf[DEF_USBD_FS_PACK_SIZE];
uint8_t hid_rx_buf[DEF_USBD_FS_PACK_SIZE];


/*********************************************************************
 * @fn      Var_Init
 *
 * @brief   Software parameter initialisation
 *
 * @return  none
 */
void Var_Init(void)
{
    uint16_t i;
    RingBuffer_Comm.LoadPtr = 0;
    RingBuffer_Comm.StopFlag = 0;
    RingBuffer_Comm.DealPtr = 0;
    RingBuffer_Comm.RemainPack = 0;
    for(i=0; i<DEF_Ring_Buffer_Max_Blks; i++)
    {
        RingBuffer_Comm.PackLen[i] = 0;
    }
}


/*********************************************************************
 * @fn      hid_send_handler
 *
 * @param   *pData - pointer of send data buffer
 *          dataLen - data length
 *
 * @brief   hid handle send operation
 *
 * @return  none
 */
void hid_send_handler( uint8_t *pData )
{

/* Determine if the USB endpoint is allowed to upload */
 if (USBHS_Endp_Busy[2] == 0x00)
 {
     /* Upload packet via USB. */
     {
         USBHS_EP2_Tx_Buf[0] = (uint8_t)HID_DATA_LEN;

         memcpy(USBHS_EP2_Tx_Buf+1, pData, HID_DATA_LEN);

//         memcpy(USBHS_EP2_Tx_Buf + Head_Pack_Len, pData, HID_DATA_LEN);

         USBHSD->UEP2_TX_DMA  = (uint32_t)(uint8_t *)USBHS_EP2_Tx_Buf;
         USBHSD->UEP2_TX_LEN  = HID_DATA_LEN + Head_Pack_Len;
         USBHS_Endp_Busy[ 2 ] |= DEF_UEP_BUSY;
         USBHSD->UEP2_TX_CTRL = ((USBHSD->UEP2_TX_CTRL) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
     }
 }

}


/*********************************************************************
 * @fn      hid_receive_handler
 *
 * @brief   handle hid receive operation
 *
 * @return  none
 */
void hid_receive_handler( uint8_t* rdBuf )
{
    volatile uint16_t hid_pkg_len = 0;

    if(RingBuffer_Comm.RemainPack)
    {
        if ( USBHS_DevSpeed == USBHS_SPEED_HIGH )
        {
            hid_pkg_len = *(uint16_t*)&Data_Buffer[(RingBuffer_Comm.DealPtr) * DEF_USBD_HS_PACK_SIZE];      // Get the valid data length
            printf("debug\r\n");
        }
        else
        {
            hid_pkg_len = (uint16_t)Data_Buffer[(RingBuffer_Comm.DealPtr) * DEF_USBD_HS_PACK_SIZE];         // Get the valid data length
            printf("debug \r\n");
        }

        if (hid_pkg_len)
        {
            if (hid_pkg_len > Data_Pack_Max_Len )
            {
                hid_pkg_len = Data_Pack_Max_Len;       // Limit the length of this transmission
            }

            for(int i=0; i<hid_pkg_len; i++)
            {
                *(rdBuf + i) = Data_Buffer[(RingBuffer_Comm.DealPtr) * DEF_USBD_HS_PACK_SIZE + 1 + i] ;

//                printf("data %d is 0x%x \r\n", i, *(rdBuf+i));
            }

//            printf("complete receive \r\n");
            /* after fetch hid received data, restore receive buffer status */
            NVIC_DisableIRQ(USBHS_IRQn);                                                  // Disable USB interrupts
            RingBuffer_Comm.RemainPack--;
            RingBuffer_Comm.DealPtr++;
            if(RingBuffer_Comm.DealPtr == DEF_Ring_Buffer_Max_Blks)
            {
                RingBuffer_Comm.DealPtr = 0;
            }
            NVIC_EnableIRQ(USBHS_IRQn);                                                   // Enable USB interrupts

        }
        else
        {
            /* drop out */
            NVIC_DisableIRQ(USBHS_IRQn);                                                  // Disable USB interrupts
            RingBuffer_Comm.RemainPack--;
            RingBuffer_Comm.DealPtr++;
            if(RingBuffer_Comm.DealPtr == DEF_Ring_Buffer_Max_Blks)
            {
               RingBuffer_Comm.DealPtr = 0;
            }
            NVIC_EnableIRQ(USBHS_IRQn);                                                   // Enable USB interrupts
        }
    }

    /* Monitor whether the remaining space is available for further downloads */
    if(RingBuffer_Comm.RemainPack < (DEF_Ring_Buffer_Max_Blks - DEF_RING_BUFFER_RESTART))
    {
        if(RingBuffer_Comm.StopFlag)
        {
            RingBuffer_Comm.StopFlag = 0;
            USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
        }
    }
}



