/*
 * gpio_cfg.h
 *
 *  Created on: Oct 20, 2023
 *      Author: pheni
 */

#ifndef USER_GPIO_CFG_H_
#define USER_GPIO_CFG_H_


/*******************************************************************************
* Include Files
*******************************************************************************/
#include "debug.h"


/*******************************************************************************
* External Variables
*******************************************************************************/
extern uint8_t led0_val;
extern uint8_t led1_val;


/*******************************************************************************
* External Functions
*******************************************************************************/
extern void pi08_gpio_init(void);
extern void pi08_preset_pin_clear();
extern void pi08_ldac_pin_clear();
extern void pi08_preset_pin_toggle(uint8_t tgValue);
extern void pi08_ldac_pin_toggle(uint8_t tgValue);
extern void pi08_led0_pin_toggle(uint8_t tgValue);
extern void pi08_led1_pin_toggle(uint8_t tgValue);


#endif /* USER_GPIO_CFG_H_ */
