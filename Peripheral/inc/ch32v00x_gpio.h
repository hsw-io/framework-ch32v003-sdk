/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00x_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V00x_GPIO_H
#define __CH32V00x_GPIO_H

#include <ch32v00x.h>
#include <stddef.h>


/* Output Maximum frequency selection */
typedef enum
{
    GPIO_SPEED_10MHz = 1,
    GPIO_SPEED_2MHz,
    GPIO_SPEED_50MHz
} GPIO_SPEED;


/* GPIO configuration mode enumeration */
typedef enum
{
    GPIO_MODE_analog_in = 0x0,
    GPIO_MODE_in_floating = 0x04,
    GPIO_MODE_in_pull_down = 0x28,
    GPIO_MODE_in_pull_up = 0x48,
    GPIO_MODE_out_open_drain = 0x14,
    GPIO_MODE_out_push_pull = 0x10,
    GPIO_MODE_alt_func_open_drain = 0x1C,
    GPIO_MODE_alt_func_push_pull = 0x18
} GPIO_MODE;


// GPIO Pin structure definition
typedef struct {
    GPIO_Regs* port;
    uint16_t num;
} Pin;


#define PIN_PA_0 (Pin){.port = GPIOA, .num = GPIO_Pin_0}
#define PIN_PA_1 (Pin){.port = GPIOA, .num = GPIO_Pin_1}
#define PIN_PA_2 (Pin){.port = GPIOA, .num = GPIO_Pin_2}
#define PIN_PA_3 (Pin){.port = GPIOA, .num = GPIO_Pin_3}
#define PIN_PA_4 (Pin){.port = GPIOA, .num = GPIO_Pin_4}
#define PIN_PA_5 (Pin){.port = GPIOA, .num = GPIO_Pin_5}
#define PIN_PA_6 (Pin){.port = GPIOA, .num = GPIO_Pin_6}
#define PIN_PA_7 (Pin){.port = GPIOA, .num = GPIO_Pin_7}

#define PIN_PC_0 (Pin){.port = GPIOC, .num = GPIO_Pin_0}
#define PIN_PC_1 (Pin){.port = GPIOC, .num = GPIO_Pin_1}
#define PIN_PC_2 (Pin){.port = GPIOC, .num = GPIO_Pin_2}
#define PIN_PC_3 (Pin){.port = GPIOC, .num = GPIO_Pin_3}
#define PIN_PC_4 (Pin){.port = GPIOC, .num = GPIO_Pin_4}
#define PIN_PC_5 (Pin){.port = GPIOC, .num = GPIO_Pin_5}
#define PIN_PC_6 (Pin){.port = GPIOC, .num = GPIO_Pin_6}
#define PIN_PC_7 (Pin){.port = GPIOC, .num = GPIO_Pin_7}

#define PIN_PD_0 (Pin){.port = GPIOD, .num = GPIO_Pin_0}
#define PIN_PD_1 (Pin){.port = GPIOD, .num = GPIO_Pin_1}
#define PIN_PD_2 (Pin){.port = GPIOD, .num = GPIO_Pin_2}
#define PIN_PD_3 (Pin){.port = GPIOD, .num = GPIO_Pin_3}
#define PIN_PD_4 (Pin){.port = GPIOD, .num = GPIO_Pin_4}
#define PIN_PD_5 (Pin){.port = GPIOD, .num = GPIO_Pin_5}
#define PIN_PD_6 (Pin){.port = GPIOD, .num = GPIO_Pin_6}
#define PIN_PD_7 (Pin){.port = GPIOD, .num = GPIO_Pin_7}


// GPIO Init structure definition
typedef struct {
    Pin pin;
    GPIO_SPEED speed;
    GPIO_MODE mode;
} GPIO_PinConfig;


/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;


/* GPIO_pins_define */
#define GPIO_Pin_0                     ((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_Pin_1                     ((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_Pin_2                     ((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_Pin_3                     ((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_Pin_4                     ((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_Pin_5                     ((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_Pin_6                     ((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_Pin_7                     ((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_Pin_All                   ((uint16_t)0xFFFF) /* All pins selected */


/* GPIO_Remap_define */
#define GPIO_Remap_SPI1                ((uint32_t)0x00000001) /* SPI1 Alternate Function mapping */
#define GPIO_PartialRemap_I2C1         ((uint32_t)0x10000002) /* I2C1 Partial Alternate Function mapping */
#define GPIO_FullRemap_I2C1            ((uint32_t)0x10400002) /* I2C1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART1      ((uint32_t)0x80000004) /* USART1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART1      ((uint32_t)0x80200000) /* USART1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_USART1          ((uint32_t)0x80200004) /* USART1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM1        ((uint32_t)0x00160040) /* TIM1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM1        ((uint32_t)0x00160080) /* TIM1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM1            ((uint32_t)0x001600C0) /* TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2        ((uint32_t)0x00180100) /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2        ((uint32_t)0x00180200) /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM2            ((uint32_t)0x00180300) /* TIM2 Full Alternate Function mapping */
#define GPIO_Remap_PA1_2               ((uint32_t)0x00008000) /* PA1 and PA2 Alternate Function mapping */
#define GPIO_Remap_ADC1_ETRGINJ        ((uint32_t)0x00200002) /* ADC1 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC1_ETRGREG        ((uint32_t)0x00200004) /* ADC1 External Trigger Regular Conversion remapping */
#define GPIO_Remap_LSI_CAL             ((uint32_t)0x00200080) /* LSI calibration Alternate Function mapping */
#define GPIO_Remap_SDI_Disable         ((uint32_t)0x00300400) /* SDI Disabled */


/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA           ((uint8_t)0x00)
#define GPIO_PortSourceGPIOC           ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD           ((uint8_t)0x03)


/* GPIO_Pin_sources */
#define GPIO_PinSource0                ((uint8_t)0x00)
#define GPIO_PinSource1                ((uint8_t)0x01)
#define GPIO_PinSource2                ((uint8_t)0x02)
#define GPIO_PinSource3                ((uint8_t)0x03)
#define GPIO_PinSource4                ((uint8_t)0x04)
#define GPIO_PinSource5                ((uint8_t)0x05)
#define GPIO_PinSource6                ((uint8_t)0x06)
#define GPIO_PinSource7                ((uint8_t)0x07)


void     gpio_deinit(GPIO_Regs *GPIOx);
void     gpio_afio_deinit(void);
void     gpio_init(GPIO_PinConfig gpio_init);
uint8_t  gpio_read_bit(Pin pin);
uint16_t gpio_read(GPIO_Regs *GPIOx);
uint8_t  gpio_read_output_bit(Pin pin);
uint16_t gpio_read_output(GPIO_Regs *GPIOx);
void     gpio_write_bit(Pin pin, BitAction action);
void     gpio_write(GPIO_Regs *GPIOx, uint16_t PortVal);
void     GPIO_PinLockConfig(GPIO_Regs *GPIOx, uint16_t pin);
void     GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void     GPIO_EventOutputCmd(FunctionalState NewState);
void     GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void     GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);


#endif /* __CH32V00x_GPIO_H */
