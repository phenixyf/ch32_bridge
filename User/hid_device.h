/*
 * hid_device.h
 *
 *  Created on: Sep 28, 2023
 *      Author: pheni
 */

#ifndef USER_HID_DEVICE_H_
#define USER_HID_DEVICE_H_

#include "debug.h"
#include "usb_desc.h"


#define SET_REPORT_DEAL_OVER          0x00
#define SET_REPORT_WAIT_DEAL          0x01
#define  HID_DATA_LEN                 63


extern uint8_t  HID_Report_Buffer[64];               // HID Report Buffer
extern volatile uint8_t HID_Set_Report_Flag;

extern void Var_Init(void);
extern void hid_receive_handler( uint8_t* rdBuf );
extern void hid_send_handler( uint8_t *pData);

extern volatile uint16_t Data_Pack_Max_Len;
extern volatile uint16_t Head_Pack_Len;

extern uint8_t hid_tx_buf[DEF_USBD_FS_PACK_SIZE];
extern uint8_t hid_rx_buf[DEF_USBD_FS_PACK_SIZE];


#endif /* USER_HID_DEVICE_H_ */
