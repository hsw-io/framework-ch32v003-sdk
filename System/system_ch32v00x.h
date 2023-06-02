/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32v00x.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : CH32V00x Device Peripheral Access Layer System Header File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __SYSTEM_CH32V00x_H
#define __SYSTEM_CH32V00x_H

#include <stdint.h>

extern uint32_t SystemCoreClock;          /* System Clock Frequency (Core Clock) */

enum SYSCLK {
    SYSCLK_48MHz_HSI,
    SYSCLK_24MHz_HSI,
    SYSCLK_8MHz_HSI,
    SYSCLK_48MHz_HSE,
    SYSCLK_24MHz_HSE,
    SYSCLK_8MHz_HSE,
};

/* System_Exported_Functions */  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#endif /*__CH32V00x_SYSTEM_H */



