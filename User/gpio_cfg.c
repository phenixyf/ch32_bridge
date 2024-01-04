/*
 * gpio_cfg.c
 *
 *  Created on: Oct 20, 2023
 *      Author: pheni
 */

#include "gpio_cfg.h"

uint8_t led0_val = 0xFF;
uint8_t led1_val = 0xFF;


void pi08_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;              // PRESET_CTR
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // LDAC_CTR
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;              // LED0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;              // LED1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void pi08_preset_pin_clear()
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, 0);
}


void pi08_preset_pin_toggle(uint8_t tgValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (tgValue == 0) ? 0 : 1);
}


void pi08_ldac_pin_clear()
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, 0);
}


void pi08_ldac_pin_toggle(uint8_t tgValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (tgValue == 0) ? 0 : 1);
}


void pi08_led0_pin_toggle(uint8_t tgValue)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_0, (tgValue == 0) ? 0 : 1);
}

void pi08_led1_pin_toggle(uint8_t tgValue)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, (tgValue == 0) ? 0 : 1);
}
