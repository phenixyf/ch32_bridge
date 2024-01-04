/*
 * protocol.h
 *
 *  Created on: Oct 4, 2023
 *      Author: pheni
 */

#ifndef USER_PROTOCOL_H_
#define USER_PROTOCOL_H_

#define     CMD_POS         0U
#define     IF_POS          1U
#define     DATA_NUM_POS    4U
#define     DATA_START_POS  5U

#define     CMD_WRITE           0x01
#define     CMD_READ            0x02
#define     CMD_SPI_FDUPLEX     0x03

#define     IF_SPI1         0x11
#define     IF_I2C1         0x12
#define     IF_GPIO         0x80

#define     PRESET_PIN      7u
#define     LDAC_PIN        6u


#endif /* USER_PROTOCOL_H_ */
