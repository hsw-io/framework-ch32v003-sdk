/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00x_i2c.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file provides all the I2C firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>

/* I2C SPE mask */
#define CTLR1_PE_Set          ((uint16_t)0x0001)
#define CTLR1_PE_Reset        ((uint16_t)0xFFFE)

/* I2C START mask */
#define CTLR1_START_Set       ((uint16_t)0x0100)
#define CTLR1_START_Reset     ((uint16_t)0xFEFF)

/* I2C STOP mask */
#define CTLR1_STOP_Set        ((uint16_t)0x0200)
#define CTLR1_STOP_Reset      ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CTLR1_ACK_Set         ((uint16_t)0x0400)
#define CTLR1_ACK_Reset       ((uint16_t)0xFBFF)

/* I2C ENGC mask */
#define CTLR1_ENGC_Set        ((uint16_t)0x0040)
#define CTLR1_ENGC_Reset      ((uint16_t)0xFFBF)

/* I2C SWRST mask */
#define CTLR1_SWRST_Set       ((uint16_t)0x8000)
#define CTLR1_SWRST_Reset     ((uint16_t)0x7FFF)

/* I2C PEC mask */
#define CTLR1_PEC_Set         ((uint16_t)0x1000)
#define CTLR1_PEC_Reset       ((uint16_t)0xEFFF)

/* I2C ENPEC mask */
#define CTLR1_ENPEC_Set       ((uint16_t)0x0020)
#define CTLR1_ENPEC_Reset     ((uint16_t)0xFFDF)

/* I2C ENARP mask */
#define CTLR1_ENARP_Set       ((uint16_t)0x0010)
#define CTLR1_ENARP_Reset     ((uint16_t)0xFFEF)

/* I2C NOSTRETCH mask */
#define CTLR1_NOSTRETCH_Set   ((uint16_t)0x0080)
#define CTLR1_NOSTRETCH_Reset ((uint16_t)0xFF7F)

/* I2C registers Masks */
#define CTLR1_CLEAR_Mask      ((uint16_t)0xFBF5)

/* I2C DMAEN mask */
#define CTLR2_DMAEN_Set       ((uint16_t)0x0800)
#define CTLR2_DMAEN_Reset     ((uint16_t)0xF7FF)

/* I2C LAST mask */
#define CTLR2_LAST_Set        ((uint16_t)0x1000)
#define CTLR2_LAST_Reset      ((uint16_t)0xEFFF)

/* I2C FREQ mask */
#define CTLR2_FREQ_Reset      ((uint16_t)0xFFC0)

/* I2C ADD0 mask */
#define OADDR1_ADD0_Set       ((uint16_t)0x0001)
#define OADDR1_ADD0_Reset     ((uint16_t)0xFFFE)

/* I2C ENDUAL mask */
#define OADDR2_ENDUAL_Set     ((uint16_t)0x0001)
#define OADDR2_ENDUAL_Reset   ((uint16_t)0xFFFE)

/* I2C ADD2 mask */
#define OADDR2_ADD2_Reset     ((uint16_t)0xFF01)

/* I2C F/S mask */
#define CKCFGR_FS_Set         ((uint16_t)0x8000)

/* I2C CCR mask */
#define CKCFGR_CCR_Set        ((uint16_t)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask             ((uint32_t)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask             ((uint32_t)0x07000000)

void i2c_deinit(I2C_Regs *i2c) {
    if (i2c == I2C1) {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
    }
}

void i2c_basics_init(I2C_Bus bus) {
    uint16_t tmpreg = 0, freqrange = 0;
    uint16_t result = 0x04;
    uint32_t pclk1 = 8000000;

    RCC_ClocksTypeDef rcc_clocks;

    tmpreg = bus.i2c->CTLR2;
    tmpreg &= CTLR2_FREQ_Reset;
    RCC_GetClocksFreq(&rcc_clocks);
    pclk1 = rcc_clocks.PCLK1_Frequency;
    freqrange = (uint16_t)(pclk1 / 1000000);
    tmpreg |= freqrange;
    bus.i2c->CTLR2 = tmpreg;

    bus.i2c->CTLR1 &= CTLR1_PE_Reset;
    tmpreg = 0;

    if (bus.speed <= 100000) {
        result = (uint16_t)(pclk1 / (bus.speed << 1));

        if (result < 0x04) {
            result = 0x04;
        }

        tmpreg |= result;
    } else {
        if (bus.duty_cycle == I2C_DUTY_CYCLE_2) {
            result = (uint16_t)(pclk1 / (bus.speed * 3));
        } else {
            result = (uint16_t)(pclk1 / (bus.speed * 25));
            result |= I2C_DUTY_CYCLE_16_9;
        }

        if ((result & CKCFGR_CCR_Set) == 0) {
            result |= (uint16_t)0x0001;
        }

        tmpreg |= (uint16_t)(result | CKCFGR_FS_Set);
    }

    bus.i2c->CKCFGR = tmpreg;
    bus.i2c->CTLR1 |= CTLR1_PE_Set;

    tmpreg = bus.i2c->CTLR1;
    tmpreg &= CTLR1_CLEAR_Mask;
    tmpreg |= (uint16_t)((uint32_t)bus.mode | bus.ack);
    bus.i2c->CTLR1 = tmpreg;

    bus.i2c->OADDR1 = (bus.acknowledged_address | bus.own_address1);
}

void i2c_init(I2C_Bus bus) {
    gpio_init((GPIO_PinConfig){
        .pin = bus.pin1,
        .mode = GPIO_MODE_alt_func_open_drain,
        .speed = GPIO_SPEED_50MHz,
    });
    gpio_init((GPIO_PinConfig){
        .pin = bus.pin2,
        .mode = GPIO_MODE_alt_func_open_drain,
        .speed = GPIO_SPEED_50MHz,
    });
    if (bus.i2c == I2C1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    }
    i2c_basics_init(bus);
    i2c_cmd(bus.i2c, ENABLE);
    if (bus.role == I2C_ROLE_Host) {
        i2c_acknowledge_config(bus.i2c, ENABLE);
    }
}

void i2c_bus_init(I2C_Bus bus) {
    bus.speed = 5000;
    bus.mode = I2C_MODE_I2C;
    bus.duty_cycle = I2C_DUTY_CYCLE_2;
    bus.own_address1 = 0;
    bus.ack = I2C_ACK_Disable;
    bus.acknowledged_address = I2C_ACKNOWLEDGED_ADDRESS_7bit;
    bus.pin1 = PIN_PC_1;
    bus.pin2 = PIN_PC_2;
}

void i2c_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_PE_Set;
    } else {
        i2c->CTLR1 &= CTLR1_PE_Reset;
    }
}

void i2c_dma_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR2 |= CTLR2_DMAEN_Set;
    } else {
        i2c->CTLR2 &= CTLR2_DMAEN_Reset;
    }
}

void i2c_dma_last_transfer_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR2 |= CTLR2_LAST_Set;
    } else {
        i2c->CTLR2 &= CTLR2_LAST_Reset;
    }
}

void i2c_generate_start(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_START_Set;
    } else {
        i2c->CTLR1 &= CTLR1_START_Reset;
    }
}

void i2c_generate_stop(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_STOP_Set;
    } else {
        i2c->CTLR1 &= CTLR1_STOP_Reset;
    }
}

void i2c_acknowledge_config(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_ACK_Set;
    } else {
        i2c->CTLR1 &= CTLR1_ACK_Reset;
    }
}

void i2c_own_address2_config(I2C_Regs *i2c, uint8_t address) {
    uint16_t tmpreg = 0;

    tmpreg = i2c->OADDR2;
    tmpreg &= OADDR2_ADD2_Reset;
    tmpreg |= (uint16_t)((uint16_t)address & (uint16_t)0x00FE);
    i2c->OADDR2 = tmpreg;
}

void i2c_dual_address_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->OADDR2 |= OADDR2_ENDUAL_Set;
    } else {
        i2c->OADDR2 &= OADDR2_ENDUAL_Reset;
    }
}

void i2c_general_call_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_ENGC_Set;
    } else {
        i2c->CTLR1 &= CTLR1_ENGC_Reset;
    }
}

void i2c_interrupts_config(I2C_Regs *i2c, uint16_t interrupt, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR2 |= interrupt;
    } else {
        i2c->CTLR2 &= (uint16_t)~interrupt;
    }
}

void i2c_send_data(I2C_Regs *i2c, uint8_t data) {
    i2c->DATAR = data;
}

uint8_t i2c_receive_data(I2C_Regs *i2c) {
    return (uint8_t)i2c->DATAR;
}

uint8_t i2c_read_byte(I2C_Regs *i2c, uint8_t from) {
    u8 temp = 0;
    i2c_generate_start(i2c, ENABLE);

    while (!i2c_check_event(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
    };
    i2c_send_7bit_address(i2c, from, I2C_DIRECTION_Receiver);

    while (!i2c_check_event(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    };
    while (i2c_get_flag_status(i2c, I2C_FLAG_RXNE) == RESET) {
        i2c_acknowledge_config(i2c, DISABLE);
    }

    temp = i2c_receive_data(i2c);
    i2c_generate_stop(i2c, ENABLE);

    return temp;
}

void i2c_send_7bit_address(I2C_Regs *i2c, uint8_t address, I2C_DIRECTION direction) {
    if (direction != I2C_DIRECTION_Transmitter) {
        address |= OADDR1_ADD0_Set;
    } else {
        address &= OADDR1_ADD0_Reset;
    }
    i2c->DATAR = address;
}

void i2c_software_reset_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_SWRST_Set;
    } else {
        i2c->CTLR1 &= CTLR1_SWRST_Reset;
    }
}

void i2c_nack_position_config(I2C_Regs *i2c, I2C_NACK_POSITION position) {
    if (position == I2C_NACK_POSITION_Next) {
        i2c->CTLR1 |= I2C_NACK_POSITION_Next;
    } else {
        i2c->CTLR1 &= I2C_NACK_POSITION_Current;
    }
}

void i2c_transmit_pec(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_PEC_Set;
    } else {
        i2c->CTLR1 &= CTLR1_PEC_Reset;
    }
}

void i2c_pec_position_config(I2C_Regs *i2c, I2C_PEC_POSITION position) {
    if (position == I2C_PEC_POSITION_Next) {
        i2c->CTLR1 |= I2C_PEC_POSITION_Next;
    } else {
        i2c->CTLR1 &= I2C_PEC_POSITION_Current;
    }
}

void i2c_calculate_pec(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_ENPEC_Set;
    } else {
        i2c->CTLR1 &= CTLR1_ENPEC_Reset;
    }
}

uint8_t i2c_get_pec(I2C_Regs *i2c) {
    return ((i2c->STAR2) >> 8);
}

void i2c_arp_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState != DISABLE) {
        i2c->CTLR1 |= CTLR1_ENARP_Set;
    } else {
        i2c->CTLR1 &= CTLR1_ENARP_Reset;
    }
}

void i2c_stretch_clock_cmd(I2C_Regs *i2c, FunctionalState NewState) {
    if (NewState == DISABLE) {
        i2c->CTLR1 |= CTLR1_NOSTRETCH_Set;
    } else {
        i2c->CTLR1 &= CTLR1_NOSTRETCH_Reset;
    }
}

void i2c_fast_mode_duty_cycle_config(I2C_Regs *i2c, I2C_DUTY_CYCLE duty_cycle) {
    if (duty_cycle != I2C_DUTY_CYCLE_16_9) {
        i2c->CKCFGR &= I2C_DUTY_CYCLE_2;
    } else {
        i2c->CKCFGR |= I2C_DUTY_CYCLE_16_9;
    }
}

ErrorStatus i2c_check_event(I2C_Regs *i2c, uint32_t event) {
    uint32_t lastevent = 0;
    uint32_t flag1 = 0, flag2 = 0;
    ErrorStatus status = NoREADY;

    flag1 = i2c->STAR1;
    flag2 = i2c->STAR2;
    flag2 = flag2 << 16;

    lastevent = (flag1 | flag2) & FLAG_Mask;

    if ((lastevent & event) == event) {
        status = READY;
    } else {
        status = NoREADY;
    }

    return status;
}

uint32_t i2c_get_last_event(I2C_Regs *i2c) {
    uint32_t flag1 = i2c->STAR1;
    uint32_t flag2 = i2c->STAR2;
    flag2 = flag2 << 16;

    uint32_t lastevent = (flag1 | flag2) & FLAG_Mask;
    return lastevent;
}

FlagStatus i2c_get_flag_status(I2C_Regs *i2c, uint32_t flag) {
    FlagStatus bitstatus = RESET;
    __IO uint32_t i2creg = 0, i2cxbase = 0;

    i2cxbase = (uint32_t)i2c;
    i2creg = flag >> 28;
    flag &= FLAG_Mask;

    if (i2creg != 0) {
        i2cxbase += 0x14;
    } else {
        flag = (uint32_t)(flag >> 16);
        i2cxbase += 0x18;
    }

    if (((*(__IO uint32_t *)i2cxbase) & flag) != (uint32_t)RESET) {
        bitstatus = SET;
    } else {
        bitstatus = RESET;
    }

    return bitstatus;
}

void i2c_clear_flag(I2C_Regs *i2c, uint32_t flag) {
    uint32_t flagpos = 0;

    flagpos = flag & FLAG_Mask;
    i2c->STAR1 = (uint16_t)~flagpos;
}

ITStatus i2c_get_interrupt_status(I2C_Regs *i2c, uint32_t interrupt) {
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;

    enablestatus = (uint32_t)(((interrupt & ITEN_Mask) >> 16) & (i2c->CTLR2));
    interrupt &= FLAG_Mask;

    if (((i2c->STAR1 & interrupt) != (uint32_t)RESET) && enablestatus) {
        bitstatus = SET;
    } else {
        bitstatus = RESET;
    }

    return bitstatus;
}

void i2c_clear_iterrupt_pending_bit(I2C_Regs *i2c, uint32_t interrupt) {
    uint32_t flagpos = 0;

    flagpos = interrupt & FLAG_Mask;
    i2c->STAR1 = (uint16_t)~flagpos;
}
