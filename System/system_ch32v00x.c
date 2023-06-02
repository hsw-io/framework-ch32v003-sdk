/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32v00x.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : CH32V00x Device Peripheral Access Layer System Source File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <ch32v00x.h>
#include "system_ch32v00x.h"
#include <stddef.h>


uint32_t SystemCoreClock = HSI_VALUE;


void default_nmi_handler(void) {
}
void (*nmi_handler_func)(void) = &default_nmi_handler;
void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void NMI_Handler(void) {
    (*nmi_handler_func)();
}


void default_hard_fault_handler(void) {
    while(1) {
    }
}
void (*hard_fault_handler_func)(void) = &default_hard_fault_handler;
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) {
    (*hard_fault_handler_func)();
}


/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash Interface,
 *        the PLL and update the SystemCoreClock variable.
 *
 * @return  none
 */
void SystemInit (void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0xFCFF0000;
  RCC->CTLR &= (uint32_t)0xFEF6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFFFEFFFF;
  RCC->INTR = 0x009F0000;
//   set_sys_clock();
}


/*********************************************************************
 * @fn      SetSysClockTo_8MHz_HSI
 *
 * @brief   Sets HSE as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_8MHz_HSI(void) {
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV3;
}


/*********************************************************************
 * @fn      SetSysClockTo_24MHz_HSI
 *
 * @brief   Sets System clock frequency to 24MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_24MHz_HSI(void) {
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
}


/*********************************************************************
 * @fn      SetSysClockTo_48MHz_HSI
 *
 * @brief   Sets System clock frequency to 48MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_48MHz_HSI(void) {
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;

    /* PLL configuration: PLLCLK = HSI * 2 = 48 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Mul2);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}


/*********************************************************************
 * @fn      SetSysClockTo_8MHz_HSE
 *
 * @brief   Sets System clock frequency to 8MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_8MHz_HSE(void) {
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* Close PA0-PA1 GPIO function */
    RCC->APB2PCENR |= RCC_AFIOEN;
    AFIO->PCFR1 |= (1<<15);

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTLR & RCC_HSERDY;
        StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    RCC->APB2PCENR |= RCC_AFIOEN;
    AFIO->PCFR1 |= (1<<15);

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
        /* Flash 0 wait state */
        FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
        FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;

        /* HCLK = SYSCLK = APB1 */
        RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV3;

        /* Select HSE as system clock source */
        RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
        RCC->CFGR0 |= (uint32_t)RCC_SW_HSE;
        /* Wait till HSE is used as system clock source */
        while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x04)
        {
        }
    }
    else
    {
        /*
         * If HSE fails to start-up, the application will have wrong clock
     * configuration. User can add here some code to deal with this error
         */
    }
}


/*********************************************************************
 * @fn      SetSysClockTo_24MHz_HSE
 *
 * @brief   Sets System clock frequency to 72MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_24MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* Close PA0-PA1 GPIO function */
    RCC->APB2PCENR |= RCC_AFIOEN;
    AFIO->PCFR1 |= (1<<15);

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTLR & RCC_HSERDY;
        StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    RCC->APB2PCENR |= RCC_AFIOEN;
    AFIO->PCFR1 |= (1<<15);

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
        /* Flash 0 wait state */
        FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
        FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;

        /* HCLK = SYSCLK = APB1 */
        RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;

        /* Select HSE as system clock source */
        RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
        RCC->CFGR0 |= (uint32_t)RCC_SW_HSE;
        /* Wait till HSE is used as system clock source */
        while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x04)
        {
        }
    }
    else
    {
        /*
         * If HSE fails to start-up, the application will have wrong clock
     * configuration. User can add here some code to deal with this error
         */
    }
}


/*********************************************************************
 * @fn      SetSysClockTo_48MHz_HSE
 *
 * @brief   Sets System clock frequency to 72MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
void SetSysClockTo_48MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* Close PA0-PA1 GPIO function */
    RCC->APB2PCENR |= RCC_AFIOEN;
    AFIO->PCFR1 |= (1<<15);

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTLR & RCC_HSERDY;
        StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
        /* Flash 0 wait state */
        FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
        FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;

        /* HCLK = SYSCLK = APB1 */
        RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;

        /* PLL configuration: PLLCLK = HSE * 2 = 48 MHz */
        RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC));
        RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE_Mul2);

        /* Enable PLL */
        RCC->CTLR |= RCC_PLLON;
        /* Wait till PLL is ready */
        while((RCC->CTLR & RCC_PLLRDY) == 0)
        {
        }
        /* Select PLL as system clock source */
        RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
        RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
        {
        }
    }
    else
    {
        /*
         * If HSE fails to start-up, the application will have wrong clock
     * configuration. User can add here some code to deal with this error
         */
    }
}


/*********************************************************************
 * @fn      set_sys_clock
 *
 * @brief   Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @param   sysclk: sets frequency and oscillator type (HSI - internal, HSE - external)
 *
 * @return  none
 */
void set_sys_clock(enum SYSCLK sysclk) {
    switch (sysclk) {
    case SYSCLK_8MHz_HSI:
        SetSysClockTo_8MHz_HSI();
        SystemCoreClock = 8000000;
        break;
    case SYSCLK_24MHz_HSI:
        SetSysClockTo_24MHz_HSI();
        SystemCoreClock = 24000000;
        break;
    case SYSCLK_48MHz_HSI:
        SetSysClockTo_48MHz_HSI();
        SystemCoreClock = 48000000;
        break;
    case SYSCLK_8MHz_HSE:
        SetSysClockTo_8MHz_HSE();
        SystemCoreClock = 8000000;
        break;
    case SYSCLK_24MHz_HSE:
        SetSysClockTo_24MHz_HSE();
        SystemCoreClock = 24000000;
        break;
    case SYSCLK_48MHz_HSE:
        SetSysClockTo_48MHz_HSE();
        SystemCoreClock = 48000000;
        break;
    }
    // If none of the define above is enabled, the HSI is used as System clock.
    // source (default after reset)
}
