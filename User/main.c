/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 PB01 bridge firmware

*/

#include "protocol.h"
#include "spi_cfg.h"
#include "i2c_cfg.h"
#include "gpio_cfg.h"

#include "hid_device.h"
#include "ch32v30x_usbhs_device.h"



/* Global typedef */

/* Global define */

/* Global Variable */



/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("This is ch32 bridge project\r\n");

	/* Variables init */
	Var_Init();

	 /* Usb Init */
	 USBHS_RCC_Init( );
	 USBHS_Device_Init( ENABLE );

    /* initial gpio */
    pi08_gpio_init();
    pi08_ldac_pin_clear();
    pi08_preset_pin_clear();
    /* initial spi1 */
    spi1_full_duplex_init();
    /* initial i2c */
    i2c1_master_init(I2C_BOUND, I2C_OWN_ADDRESS);


    /* Initial hid buffer */
    memset(hid_rx_buf, 0x00, sizeof(hid_rx_buf));
    memset(hid_tx_buf, 0x00, sizeof(hid_tx_buf));

	while(1)
    {
	    if(USBHS_DevEnumStatus)
        {
            hid_receive_handler(hid_rx_buf);


            switch(hid_rx_buf[CMD_POS])
            {
                case CMD_WRITE:
                {
                    switch(hid_rx_buf[IF_POS])
                    {
                        case IF_SPI1:
                        {
                            spix_write_array(SPI1, hid_rx_buf[DATA_NUM_POS], &hid_rx_buf[DATA_START_POS]);

                            led1_val = ~led1_val;
                            pi08_led1_pin_toggle(led1_val);
                            break;
                        }   // end case IF_SPI

                        case IF_I2C1:
                        {
                            if(i2c1_write_array(hid_rx_buf[2], &hid_rx_buf[DATA_START_POS], hid_rx_buf[DATA_NUM_POS]) == 0)
                            {
                                I2C_DeInit(I2C1);
                                led1_val = ~led1_val;
                                pi08_led1_pin_toggle(led1_val);
                                i2c1_master_init(I2C_BOUND, I2C_OWN_ADDRESS);
                            }

                            led1_val = ~led1_val;
                            pi08_led1_pin_toggle(led1_val);
                            break;
                        }   // end case IF_I2C

                        case IF_GPIO:
                        {
                            if(hid_rx_buf[2] == PRESET_PIN)
                            {
                                pi08_preset_pin_toggle(hid_rx_buf[5]);
                            }else if (hid_rx_buf[2] == LDAC_PIN)
                            {
                                pi08_ldac_pin_toggle(hid_rx_buf[5]);
                            }

                            led1_val = ~led1_val;
                            pi08_led1_pin_toggle(led1_val);
                            break;
                        }   // end case IF_I2C

                        default:
                            printf("input IF type error\r\n");
                            break;
                    }
                    break;
                }   // end case CMD_WRITE

                case CMD_READ:
                {
                    switch(hid_rx_buf[IF_POS])
                    {
                        case IF_SPI1:
                        {
                            break;
                        }   // end case IF_SPI

                        case IF_I2C1:
                        {
                            if(i2c1_write_array(hid_rx_buf[2], &hid_rx_buf[DATA_START_POS], 1) == 0)
                            {
                                I2C_DeInit(I2C1);
                                led1_val = ~led1_val;
                                pi08_led1_pin_toggle(led1_val);
                                i2c1_master_init(I2C_BOUND, I2C_OWN_ADDRESS);
                            }

                            if(i2c1_read_array(hid_rx_buf[2], hid_tx_buf, hid_rx_buf[DATA_NUM_POS]) == 0)
                            {
                                I2C_DeInit(I2C1);
                                led1_val = ~led1_val;
                                pi08_led1_pin_toggle(led1_val);
                                i2c1_master_init(I2C_BOUND, I2C_OWN_ADDRESS);
                            }

                            hid_send_handler(hid_tx_buf);

                            led1_val = ~led1_val;
                            pi08_led1_pin_toggle(led1_val);
                            break;
                        }   // end case IF_I2C

                        default:
                            printf("input IF type error\r\n");
                            break;
                    }
                    break;
                }   // end case CMD_READ

                case CMD_SPI_FDUPLEX:
                {
                    switch(hid_rx_buf[IF_POS])
                    {
                        case IF_SPI1:
                        {
                            spix_read_array(SPI1, hid_rx_buf[DATA_NUM_POS], &hid_rx_buf[DATA_START_POS], hid_tx_buf);
                            hid_send_handler(hid_tx_buf);

                            led1_val = ~led1_val;
                            pi08_led1_pin_toggle(led1_val);
                            break;
                        }   // end case IF_SPI
                    }

                    break;
                }// end case CMD_SPI_FDUPLEX
                default:
                    break;
            }

            memset(hid_rx_buf, 0x00, sizeof(hid_rx_buf));
            memset(hid_tx_buf, 0x00, sizeof(hid_tx_buf));

        }
	}




}

