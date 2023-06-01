/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00x_gpio.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file provides all the GPIO firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <ch32v00x_gpio.h>
#include <ch32v00x_rcc.h>

/* MASK */
#define LSB_MASK                  ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK      ((uint32_t)0x000F0000)
#define DBGAFR_SDI_MASK           ((uint32_t)0xF8FFFFFF)
#define DBGAFR_LOCATION_MASK      ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK       ((uint32_t)0x00100000)

/*********************************************************************
 * @fn      gpio_deinit
 *
 * @brief   Deinitializes the port peripheral registers to their default
 *        reset values.
 *
 * @param   port: where x can be (A..G) to select the GPIO peripheral.
 *
 * @return  none
 */
void gpio_deinit(GPIOPort *port)
{
    if(port == GPIOA)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
    }
    else if(port == GPIOC)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, DISABLE);
    }
    else if(port == GPIOD)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, DISABLE);
    }
}

/*********************************************************************
 * @fn      gpio_afio_deinit
 *
 * @brief   Deinitializes the Alternate Functions (remap, event control
 *        and EXTI configuration) registers to their default reset values.
 *
 * @return  none
 */
void gpio_afio_deinit(void)
{
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
}

/*********************************************************************
 * @fn      gpio_init
 *
 * @brief   configure GPIO pin
 *
 * @param   gpio  GPIOInit structure that
 *        contains the configuration information for the specified GPIO peripheral.
 *
 * @return  none
 */
void gpio_init(GPIOInit gpio)
{
    uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
    uint32_t tmpreg = 0x00, pinmask = 0x00;

    currentmode = ((uint32_t)gpio.mode) & ((uint32_t)0x0F);

    if((((uint32_t)gpio.mode) & ((uint32_t)0x10)) != 0x00)
    {
        currentmode |= (uint32_t)gpio.speed;
    }

    if(((uint32_t)gpio.pin.num & ((uint32_t)0x00FF)) != 0x00)
    {
        tmpreg = gpio.pin.port->CFGLR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = ((uint32_t)0x01) << pinpos;
            currentpin = (gpio.pin.num) & pos;

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(gpio.mode == GPIO_MODE_in_pull_down)
                {
                    gpio.pin.port->BCR = (((uint32_t)0x01) << pinpos);
                }
                else
                {
                    if(gpio.mode == GPIO_MODE_in_pull_up)
                    {
                        gpio.pin.port->BSHR = (((uint32_t)0x01) << pinpos);
                    }
                }
            }
        }
        gpio.pin.port->CFGLR = tmpreg;
    }

    if(gpio.pin.num > 0x00FF)
    {
        tmpreg = gpio.pin.port->CFGHR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = (((uint32_t)0x01) << (pinpos + 0x08));
            currentpin = ((gpio.pin.num) & pos);

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(gpio.mode == GPIO_MODE_in_pull_down)
                {
                    gpio.pin.port->BCR = (((uint32_t)0x01) << (pinpos + 0x08));
                }

                if(gpio.mode == GPIO_MODE_in_pull_up)
                {
                    gpio.pin.port->BSHR = (((uint32_t)0x01) << (pinpos + 0x08));
                }
            }
        }
        gpio.pin.port->CFGHR = tmpreg;
    }
}


/*********************************************************************
 * @fn      gpio_read_bit
 *
 * @brief   Read bit from specified port
 *
 * @param   pin: specifies the port and bit to read
 *
 * @return  The input port pin value.
 */
uint8_t gpio_read_bit(Pin pin)
{
    uint8_t bitstatus = 0x00;

    if((pin.port->INDR & pin.num) != (uint32_t)Bit_RESET) {
        bitstatus = (uint8_t)Bit_SET;
    }
    else {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      gpio_read
 *
 * @brief   Reads the specified GPIO input data port.
 *
 * @param   port: where x can be (A..G) to select the GPIO peripheral.
 *
 * @return  The input port pin value.
 */
uint16_t gpio_read(GPIOPort *port)
{
    return ((uint16_t)port->INDR);
}

/*********************************************************************
 * @fn      gpio_read_output_bit
 *
 * @brief   Reads the specified output data port bit.
 *
 * @param   pin specifies the port and bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
uint8_t gpio_read_output_bit(Pin pin) {
    uint8_t bitstatus = 0x00;

    if((pin.port->OUTDR & pin.num) != (uint32_t)Bit_RESET) {
        bitstatus = (uint8_t)Bit_SET;
    }
    else {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      gpio_read_output
 *
 * @brief   Reads the specified GPIO output data port.
 *
 * @param   port - where x can be (A..G) to select the GPIO peripheral.
 *
 * @return  GPIO output port pin value.
 */
uint16_t gpio_read_output(GPIOPort *port)
{
    return ((uint16_t)port->OUTDR);
}

/*********************************************************************
 * @fn      gpio_write_bit
 *
 * @brief   Sets or clears the selected data port bit.
 *
 * @param   pin - specifies the port bit to be written.
 * @param   action - specifies the value to be written to the selected bit.
 *            Bit_SetL - to clear the port pin.
 *            Bit_SetH - to set the port pin.
 *
 * @return  none
 */
void gpio_write_bit(Pin pin, BitAction action)
{
    if(action != Bit_RESET) {
        pin.port->BSHR = pin.num;
    }
    else {
        pin.port->BCR = pin.num;
    }
}

/*********************************************************************
 * @fn      gpio_write
 *
 * @brief   Writes data to the specified GPIO data port.
 *
 * @param   port: where x can be (A..G) to select the GPIO peripheral.
 * @param   value: specifies the value to be written to the port output data register.
 *
 * @return  none
 */
void gpio_write(GPIOPort *port, uint16_t value)
{
    port->OUTDR = value;
}

/*********************************************************************
 * @fn      GPIO_PinLockConfig
 *
 * @brief   Locks GPIO Pins configuration registers.
 *
 * @param   port - where x can be (A..G) to select the GPIO peripheral.
 *          pin - specifies the port bit to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_PinLockConfig(GPIOPort *port, uint16_t pin)
{
    uint32_t tmp = 0x00010000;

    tmp |= pin;
    port->LCKR = tmp;
    port->LCKR = pin;
    port->LCKR = tmp;
    tmp = port->LCKR;
    tmp = port->LCKR;
}

/*********************************************************************
 * @fn      GPIO_PinRemapConfig
 *
 * @brief   Changes the mapping of the specified pin.
 *
 * @param   GPIO_Remap - selects the pin to remap.
 *            GPIO_Remap_SPI1 - SPI1 Alternate Function mapping
 *            GPIO_PartialRemap_I2C1 - I2C1 Partial Alternate Function mapping
 *            GPIO_PartialRemap_I2C1 - I2C1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_USART1 - USART1 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_USART1 - USART1 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_USART1 - USART1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_TIM1 - TIM1 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_TIM1 - TIM1 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_TIM1 - TIM1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_TIM2 - TIM2 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_TIM2 - TIM2 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_TIM2 - TIM2 Full Alternate Function mapping
 *            GPIO_Remap_PA12 - PA12 Alternate Function mapping
 *            GPIO_Remap_ADC1_ETRGINJ - ADC1 External Trigger Injected Conversion remapping
 *            GPIO_Remap_ADC1_ETRGREG - ADC1 External Trigger Regular Conversion remapping
 *            GPIO_Remap_LSI_CAL - LSI calibration Alternate Function mapping
 *            GPIO_Remap_SDI_Disable - SDI Disabled
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

    tmpreg = AFIO->PCFR1;

    tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
    tmp = GPIO_Remap & LSB_MASK;

    if((GPIO_Remap & 0x10000000) == 0x10000000)
    {
        tmpreg &= ~((1<<1) | (1<<22));
        tmpreg |= ~DBGAFR_SDI_MASK;
        if(NewState != DISABLE)
        {
            tmpreg |= (GPIO_Remap & 0xEFFFFFFF);
        }

    }
    else if((GPIO_Remap & 0x80000000) == 0x80000000)
    {
        tmpreg &= ~((1<<2) | (1<<21));
        tmpreg |= ~DBGAFR_SDI_MASK;
        if(NewState != DISABLE)
        {
            tmpreg |= (GPIO_Remap & 0x7FFFFFFF);
        }

    }
    else if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK))/* SDI */
    {
        tmpreg &= DBGAFR_SDI_MASK;
        AFIO->PCFR1 &= DBGAFR_SDI_MASK;

        if(NewState != DISABLE)
        {
            tmpreg |= (tmp << ((GPIO_Remap >> 0x15) * 0x10));
        }
    }
    else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK)/* [15:0] 2bit */
    {
        tmp1 = ((uint32_t)0x03) << tmpmask;
        tmpreg &= ~tmp1;
        tmpreg |= ~DBGAFR_SDI_MASK;

        if(NewState != DISABLE)
        {
            tmpreg |= (tmp << ((GPIO_Remap >> 0x15) * 0x10));
        }
    }
    else/* [31:0] 1bit */
    {
        tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15) * 0x10));
        tmpreg |= ~DBGAFR_SDI_MASK;

        if(NewState != DISABLE)
        {
            tmpreg |= (tmp << ((GPIO_Remap >> 0x15) * 0x10));
        }
    }


     AFIO->PCFR1 = tmpreg;
}

/*********************************************************************
 * @fn      GPIO_EXTILineConfig
 *
 * @brief   Selects the GPIO pin used as EXTI Line.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source for EXTI lines.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..G).
 *          GPIO_PinSource - specifies the EXTI line to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..7).
 *
 * @return  none
 */
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)(3<<(GPIO_PinSource<<1)));
    AFIO->EXTICR &= ~tmp;
    AFIO->EXTICR |= ((uint32_t)(GPIO_PortSource<<(GPIO_PinSource<<1)));
}
