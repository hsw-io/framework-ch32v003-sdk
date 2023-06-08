/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00x_i2c.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file contains all the functions prototypes for the
 *                      I2C firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V00x_I2C_H
#define __CH32V00x_I2C_H

#include <ch32v00x.h>
#include <ch32v00x_gpio.h>

// I2C bus configuration
typedef struct
{
    I2C_Regs *i2c;  // pointer to I2C perif registers
    uint32_t speed; // must be lower than 400kHz
    I2C_MODE mode;
    I2C_DUTY_CYCLE duty_cycle;
    uint16_t own_address1; // 7-bit or 10-bit address
    I2C_ACK ack;
    I2C_ACKNOWLEDGED_ADDRESS acknowledged_address;
    Pin pin1;
    Pin pin2;
    I2C_ROLE role;
} I2C_Bus;

// I2C bus role
typedef enum {
    I2C_ROLE_Host,
    I2C_ROLE_Slave,
} I2C_ROLE;

// TODO: check why here is the only one mode
//       probably here should be at least another one: SMBus
// I2C mode
typedef enum {
    I2C_MODE_I2C = 0x0000,
} I2C_MODE;

// I2C duty cycle in fast mode
typedef enum {
    I2C_DUTY_CYCLE_16_9 = 0x4000, // I2C fast mode Tlow/Thigh = 16/9
    I2C_DUTY_CYCLE_2 = 0xBFFF,    // I2C fast mode Tlow/Thigh = 2
} I2C_DUTY_CYCLE;

// I2C acknowledgement
typedef enum {
    I2C_ACK_Enable = 0x0400,
    I2C_ACK_Disable = 0x0000,
} I2C_ACK;

// I2C transfer direction
typedef enum {
    I2C_DIRECTION_Transmitter,
    I2C_DIRECTION_Receiver,
} I2C_DIRECTION;

// I2C acknowledged address
typedef enum {
    I2C_ACKNOWLEDGED_ADDRESS_7bit = 0x4000,
    I2C_ACKNOWLEDGED_ADDRESS_10bit = 0xC000,
} I2C_ACKNOWLEDGED_ADDRESS;

// TODO: concider to delete following definitions
// I2C registers
#define I2C_Register_CTLR1  ((uint8_t)0x00)
#define I2C_Register_CTLR2  ((uint8_t)0x04)
#define I2C_Register_OADDR1 ((uint8_t)0x08)
#define I2C_Register_OADDR2 ((uint8_t)0x0C)
#define I2C_Register_DATAR  ((uint8_t)0x10)
#define I2C_Register_STAR1  ((uint8_t)0x14)
#define I2C_Register_STAR2  ((uint8_t)0x18)
#define I2C_Register_CKCFGR ((uint8_t)0x1C)

// I2C PEC position
typedef enum {
    I2C_PEC_POSITION_Next = 0x0800,    // indicates that the next byte is PEC
    I2C_PEC_POSITION_Current = 0xF7FF, // indicates that current byte is PEC
} I2C_PEC_POSITION;

// I2C NACK position
typedef enum {
    I2C_NACK_POSITION_Next = 0x0800,    // indicates that the next byte will be the last received byte
    I2C_NACK_POSITION_Current = 0xF7FF, // indicates that current byte is the last received byte
} I2C_NACK_POSITION;

// I2C interrupts definition
#define I2C_IT_BUF                                        ((uint16_t)0x0400)
#define I2C_IT_EVT                                        ((uint16_t)0x0200)
#define I2C_IT_ERR                                        ((uint16_t)0x0100)

// I2C interrupts definition
#define I2C_IT_PECERR                                     ((uint32_t)0x01001000)
#define I2C_IT_OVR                                        ((uint32_t)0x01000800)
#define I2C_IT_AF                                         ((uint32_t)0x01000400)
#define I2C_IT_ARLO                                       ((uint32_t)0x01000200)
#define I2C_IT_BERR                                       ((uint32_t)0x01000100)
#define I2C_IT_TXE                                        ((uint32_t)0x06000080)
#define I2C_IT_RXNE                                       ((uint32_t)0x06000040)
#define I2C_IT_STOPF                                      ((uint32_t)0x02000010)
#define I2C_IT_ADD10                                      ((uint32_t)0x02000008)
#define I2C_IT_BTF                                        ((uint32_t)0x02000004)
#define I2C_IT_ADDR                                       ((uint32_t)0x02000002)
#define I2C_IT_SB                                         ((uint32_t)0x02000001)

// SR2 register flags
#define I2C_FLAG_DUALF                                    ((uint32_t)0x00800000)
#define I2C_FLAG_GENCALL                                  ((uint32_t)0x00100000)
#define I2C_FLAG_TRA                                      ((uint32_t)0x00040000)
#define I2C_FLAG_BUSY                                     ((uint32_t)0x00020000)
#define I2C_FLAG_MSL                                      ((uint32_t)0x00010000)

// SR1 register flags
#define I2C_FLAG_PECERR                                   ((uint32_t)0x10001000)
#define I2C_FLAG_OVR                                      ((uint32_t)0x10000800)
#define I2C_FLAG_AF                                       ((uint32_t)0x10000400)
#define I2C_FLAG_ARLO                                     ((uint32_t)0x10000200)
#define I2C_FLAG_BERR                                     ((uint32_t)0x10000100)
#define I2C_FLAG_TXE                                      ((uint32_t)0x10000080)
#define I2C_FLAG_RXNE                                     ((uint32_t)0x10000040)
#define I2C_FLAG_STOPF                                    ((uint32_t)0x10000010)
#define I2C_FLAG_ADD10                                    ((uint32_t)0x10000008)
#define I2C_FLAG_BTF                                      ((uint32_t)0x10000004)
#define I2C_FLAG_ADDR                                     ((uint32_t)0x10000002)
#define I2C_FLAG_SB                                       ((uint32_t)0x10000001)

// --- I2C Master Events (Events grouped in order of communication) ---

// Start communicate
// -----------------
// After master use i2c_generate_start() function sending the START condition,the master
// has to wait for event 5 (the Start condition has been correctly
// released on the I2C bus).

// EVT5
#define I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001) // BUSY, MSL and SB flag

// Address Acknowledge
// -------------------
// When start condition correctly released on the bus(check EVT5), the
// master use i2c_send_7bit_address() function sends the address of the slave(s) with which it will communicate
// it also determines master as Transmitter or Receiver. Then the master has to wait that a slave acknowledges
// his address. If an acknowledge is sent on the bus, one of the following events will be set:
//  1) In case of Master Receiver (7-bit addressing):
//     the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED event is set.
//  2) In case of Master Transmitter (7-bit addressing):
//     the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED is set
//  3) In case of 10-Bit addressing mode, the master (after generating the START
//  and checking on EVT5) use i2c_send_data() function send the header of 10-bit addressing mode.
//  Then master wait EVT9. EVT9 means that the 10-bit addressing header has been correctly sent
//  on the bus. Then master should use the function i2c_send_7bit_address() to send the second part
//  of the 10-bit address (LSB) . Then master should wait for event 6.

// EVT6
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082) // BUSY, MSL, ADDR, TXE and TRA flags
#define I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((uint32_t)0x00030002) // BUSY, MSL and ADDR flags
// EVT9
#define I2C_EVENT_MASTER_MODE_ADDRESS10                   ((uint32_t)0x00030008) // BUSY, MSL and ADD10 flags

// Communication events
// --------------------
// If START condition has generated and slave address been acknowledged,
// then the master has to check one of the following events for
// communication procedures:
// 1) Master Receiver mode: The master has to wait on the event EVT7 then use
//    i2c_receive_data() function to read the data received from the slave.
// 2) Master Transmitter mode: The master use i2c_send_data() function to send data
//    then to wait on event EVT8 or EVT8_2.
//    These two events are similar:
//     - EVT8 means that the data has been written in the data register and is
//       being shifted out.
//     - EVT8_2 means that the data has been physically shifted out and output
//       on the bus.
//     In most cases, using EVT8 is sufficient for the application.
//     Using EVT8_2  will leads to a slower communication  speed but will more reliable.
//     EVT8_2 is also more suitable than EVT8 for testing on the last data transmission
//  Note:
//  In case the user software does not guarantee that this event EVT7 is managed before
//  the current byte end of transfer, then user may check on I2C_EVENT_MASTER_BYTE_RECEIVED
//  and I2C_FLAG_BTF flag at the same time. But in this case the communication may be slower.

// Master Receive mode
// EVT7
#define I2C_EVENT_MASTER_BYTE_RECEIVED                    ((uint32_t)0x00030040) // BUSY, MSL and RXNE flags

// Master Transmitter mode
// EVT8
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                ((uint32_t)0x00070080) // TRA, BUSY, MSL, TXE flags
// EVT8_2
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084) // TRA, BUSY, MSL, TXE and BTF flags

// --- I2C Slave Events (Events grouped in order of communication) ---

// Start Communicate events
// ------------------------
// Wait on one of these events at the start of the communication. It means that
// the I2C peripheral detected a start condition of master device generate on the bus.
// If the acknowledge feature is enabled through function i2c_acknowledge_config(),
// the peripheral generates an ACK condition on the bus.
//
// a) In normal case (only one address managed by the slave), when the address
//   sent by the master matches the own address of the peripheral (configured by
//   own_address1 field) the I2C_EVENT_SLAVE_XXX_ADDRESS_MATCHED event is set
//   (where XXX could be TRANSMITTER or RECEIVER).
// b) In case the address sent by the master matches the second address of the
//   peripheral (configured by the function i2c_own_address2_config() and enabled
//   by the function i2c_dual_address_cmd()) the events I2C_EVENT_SLAVE_XXX_SECONDADDRESS_MATCHED
//   (where XXX could be TRANSMITTER or RECEIVER) are set.
// c) In case the address sent by the master is General Call (address 0x00) and
//   if the General Call is enabled for the peripheral (using function i2c_general_call_cmd())
//   the following event is set I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED.

// EVT1
// a) Case of One Single Address managed by the slave
#define I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((uint32_t)0x00020002) // BUSY and ADDR flags
#define I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((uint32_t)0x00060082) // TRA, BUSY, TXE and ADDR flags
// b) Case of Dual address managed by the slave
#define I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED    ((uint32_t)0x00820000) // DUALF and BUSY flags
#define I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED ((uint32_t)0x00860080) // DUALF, TRA, BUSY and TXE flags
// c) Case of General Call enabled for the slave
#define I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED        ((uint32_t)0x00120000) // GENCALL and BUSY flags

// Communication events
// --------------------
// Wait on one of these events when EVT1 has already been checked:
//
// - Slave Receiver mode:
//     - EVT2 -- The device is expecting to receive a data byte.
//     - EVT4 -- The device is expecting the end of the communication: master
//       sends a stop condition and data transmission is stopped.
// - Slave Transmitter mode:
//    - EVT3 -- When a byte has been transmitted by the slave and the Master is expecting
//      the end of the byte transmission. The two events I2C_EVENT_SLAVE_BYTE_TRANSMITTED and
//      I2C_EVENT_SLAVE_BYTE_TRANSMITTING are similar. If the user software doesn't guarantee
//      the EVT3 is managed before the current byte end of transfer The second one can optionally
//      be used.
//    - EVT3_2 -- When the master sends a NACK to tell slave device that data transmission
//      shall end . The slave device has to stop sending
//      data bytes and wait a Stop condition from bus.
//  Note:
//  If the user software does not guarantee that the event 2 is
//  managed before the current byte end of transfer, User may check on I2C_EVENT_SLAVE_BYTE_RECEIVED
//  and I2C_FLAG_BTF flag at the same time.
//  In this case the communication will be slower.

// Slave Receiver mode
// EVT2
#define I2C_EVENT_SLAVE_BYTE_RECEIVED                     ((uint32_t)0x00020040) // BUSY and RXNE flags
// EVT4
#define I2C_EVENT_SLAVE_STOP_DETECTED                     ((uint32_t)0x00000010) // STOPF flag

// Slave Transmitter mode
// EVT3
#define I2C_EVENT_SLAVE_BYTE_TRANSMITTED                  ((uint32_t)0x00060084) // TRA, BUSY, TXE and BTF flags
#define I2C_EVENT_SLAVE_BYTE_TRANSMITTING                 ((uint32_t)0x00060080) // TRA, BUSY and TXE flags
// EVT3_2
#define I2C_EVENT_SLAVE_ACK_FAILURE                       ((uint32_t)0x00000400) // AF flag

/// @brief Deinitializes the i2c peripheral registers to their default
///  reset values.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @return none
void i2c_deinit(I2C_Regs *i2c);

/// @brief Init only the i2c peripheral according to the specified parameters.
/// @param bus I2C_Bus structure that contains the pointer and configuration
///        for the specified I2C peripheral.
/// @return none
void i2c_basics_init(I2C_Bus bus);

/// @brief Init the i2c peripheral and GPIO pins used by bus
/// @param bus I2C_Bus stucture
/// @return none
void i2c_init(I2C_Bus bus);

/// @brief Fills each bus member with its default value.
/// @param bus pointer to an I2C_Bus structure which
///  will be initialized.
/// @return none
void i2c_bus_init(I2C_Bus I2C_InitStruct);

/// @brief Enables or disables the specified I2C peripheral.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Enables or disables the specified I2C DMA requests.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_dma_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Generates i2c communication START condition.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_dma_last_transfer_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Generates i2c communication START condition.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_generate_start(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Generates i2c communication STOP condition.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_generate_stop(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Enables or disables the specified I2C acknowledge feature.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_acknowledge_config(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Configures the specified I2C own address2.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param address specifies the 7bit I2C own address2.
/// @return none
void i2c_own_address2_config(I2C_Regs *i2c, uint8_t Address);

/// @brief Enables or disables the specified I2C dual addressing mode.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_dual_address_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Enables or disables the specified I2C general call feature.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_general_call_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Enables or disables the specified I2C interrupts.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param interrupt specifies the I2C interrupts sources to be enabled or disabled.
///  I2C_IT_BUF - Buffer interrupt mask.
///  I2C_IT_EVT - Event interrupt mask.
///  I2C_IT_ERR - Error interrupt mask.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_interrupts_config(I2C_Regs *i2c, uint16_t I2C_IT, FunctionalState NewState);

/// @brief Sends a data byte through the i2c peripheral.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param data Byte to be transmitted.
/// @return none
void i2c_send_data(I2C_Regs *i2c, uint8_t Data);

/// @brief Returns the most recent received data by the i2c peripheral.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @return The value of the received data.
uint8_t i2c_receive_data(I2C_Regs *i2c);

/// @brief return the most recent byte data received via the i2c bus
/// @param i2c pointer to I2C peripheral
/// @param from slave address
/// @return received byte
uint8_t i2c_read_byte(I2C_Regs *i2c, uint8_t from);

/// @brief Transmits the address byte to select the slave device.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param address specifies the slave address which will be transmitted.
/// @param direction specifies whether the I2C device will be a
///  Transmitter or a Receiver.
/// @return  none
void i2c_send_7bit_address(I2C_Regs *i2c, uint8_t address, I2C_DIRECTION direction);

/// @brief Enables or disables the specified I2C software reset.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState - ENABLE or DISABLE.
/// @return none
void i2c_software_reset_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Selects the specified I2C NACK position in master receiver mode.
/// Note:
///    This function configures the same bit (POS) as i2c_pec_position_config()
///    but is intended to be used in I2C mode while i2c_pec_position_config()
///    is intended to used in SMBUS mode.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param position specifies the NACK position.
/// @return none
void i2c_nack_position_config(I2C_Regs *i2c, I2C_NACK_POSITION I2C_NACKPosition);

/// @brief Enables or disables the specified I2C PEC transfer.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_transmit_pec(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Selects the specified I2C PEC position.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param position - specifies the PEC position.
/// @return none
void i2c_pec_position_config(I2C_Regs *i2c, I2C_PEC_POSITION position);

/// @brief Enables or disables the PEC value calculation of the transferred bytes.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_calculate_pec(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Returns the PEC value for the specified I2C.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @return The PEC value.
uint8_t i2c_get_pec(I2C_Regs *i2c);

/// @brief Enables or disables the specified I2C ARP.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return The PEC value.
void i2c_arp_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Enables or disables the specified I2C Clock stretching.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param NewState ENABLE or DISABLE.
/// @return none
void i2c_stretch_clock_cmd(I2C_Regs *i2c, FunctionalState NewState);

/// @brief Selects the specified I2C fast mode duty cycle.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param duty_cycle specifies the fast mode duty cycle.
/// @return none
void i2c_fast_mode_duty_cycle_config(I2C_Regs *i2c, I2C_DUTY_CYCLE duty_cycle);

// I2C State Monitoring Functions
// ------------------------------
// This I2C driver provides three different ways for I2C state monitoring
// profit the application requirements and constraints:
//
// a) First way:
//    Using i2c_check_event() function:
//    It compares the status registers (STARR1 and STAR2) content to a given event
//    (can be the combination of more flags).
//    If the current status registers includes the given flags will return SUCCESS.
//    and if the current status registers miss flags will returns ERROR.
//    - When to use:
//      - This function is suitable for most applications as well as for startup
//        activity since the events are fully described in the product reference manual
//        (CH32V03RM).
//      - It is also suitable for users who need to define their own events.
//    - Limitations:
//      - If an error occurs besides to the monitored error,
//        the i2c_check_event() function may return SUCCESS despite the communication
//        in corrupted state.  it is suggeted to use error interrupts to monitor the error
//        events and handle them in IRQ handler.
//
//        Note:
//        The following functions are recommended for error management:
//          - i2c_interrupts_config() main function of configure and enable the error interrupts.
//          - I2Cx_ER_IRQHandler() will be called when the error interrupt happen.
//            Where x is the peripheral instance (I2C1, I2C2 ...)
//          - I2Cx_ER_IRQHandler() will call i2c_get_flag_status() or i2c_get_interrupt_status() functions
//            to determine which error occurred.
//          - i2c_clear_flag() \ i2c_clear_iterrupt_pending_bit() \ i2c_software_reset_cmd()
//            \ I2C_GenerateStop() will be use to clear the error flag and source,
//            and return to correct communication status.
//
//  b) Second way:
//     Using the function to get a single word(uint32_t) composed of status register 1 and register 2.
//     (Status Register 2 value is shifted left by 16 bits and concatenated to Status Register 1).
//     - When to use:
//       - This function is suitable for the same applications above but it
//         don't have the limitations of i2c_get_flag_status() function.
//         The returned value could be compared to events already defined in the
//         library (CH32V00x_i2c.h) or to custom values defined by user.
//       - This function can be used to monitor the status of multiple flags simultaneously.
//       - Contrary to the i2c_check_event () function, this function can choose the time to
//         accept the event according to the user's needs (when all event flags are set and
//         no other flags are set, or only when the required flags are set)
//     - Limitations:
//       - User may need to define his own events.
//       - Same remark concerning the error management is applicable for this
//         function if user decides to check only regular communication flags (and
//         ignores error flags).
//
//  c) Third way:
//     Using the function i2c_get_flag_status() get the status of
//     one single flag.
//     - When to use:
//        - This function could be used for specific applications or in debug phase.
//        - It is suitable when only one flag checking is needed.
//     - Limitations:
//        - Call this function to access the status register. Some flag bits may be cleared.
//        - Function may need to be called twice or more in order to monitor one single event.

/// @brief a) Basic state monitoring (First way)
///  Checks whether the last i2c Event is equal to the one passed as parameter.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param event specifies the event to be checked.
///  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED - EVT1.
///  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED - EVT1.
///  I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED - EVT1.
///  I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED - EVT1.
///  I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED - EVT1.
///  I2C_EVENT_SLAVE_BYTE_RECEIVED - EVT2.
///  (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF) - EVT2.
///  (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL) - EVT2.
///  I2C_EVENT_SLAVE_BYTE_TRANSMITTED - EVT3.
///  (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF) - EVT3.
///  (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) - EVT3.
///  I2C_EVENT_SLAVE_ACK_FAILURE - EVT3_2.
///  I2C_EVENT_SLAVE_STOP_DETECTED - EVT4.
///  I2C_EVENT_MASTER_MODE_SELECT - EVT5.
///  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED - EVT6.
///  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED - EVT6.
///  I2C_EVENT_MASTER_BYTE_RECEIVED - EVT7.
///  I2C_EVENT_MASTER_BYTE_TRANSMITTING - EVT8.
///  I2C_EVENT_MASTER_BYTE_TRANSMITTED - EVT8_2.
///  I2C_EVENT_MASTER_MODE_ADDRESS10 - EVT9.
/// @return the last i2c event is READY or NoREADY.
ErrorStatus i2c_check_event(I2C_Regs *I2Cx, uint32_t I2C_EVENT);

/// @brief b) Advanced state monitoring (Second way)
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @return the last i2c Event
uint32_t i2c_get_last_event(I2C_Regs *I2Cx);

/// @brief c) Flag-based state monitoring (Third way)
///  Checks whether the passed flag is set or not.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param flag specifies the flag to check.
///  I2C_FLAG_DUALF - Dual flag (Slave mode).
///  I2C_FLAG_GENCALL - General call header flag (Slave mode).
///  I2C_FLAG_TRA - Transmitter/Receiver flag.
///  I2C_FLAG_BUSY - Bus busy flag.
///  I2C_FLAG_MSL - Master/Slave flag.
///  I2C_FLAG_PECERR - PEC error in reception flag.
///  I2C_FLAG_OVR - Overrun/Underrun flag (Slave mode).
///  I2C_FLAG_AF - Acknowledge failure flag.
///  I2C_FLAG_ARLO - Arbitration lost flag (Master mode).
///  I2C_FLAG_BERR - Bus error flag.
///  I2C_FLAG_TXE - data register empty flag (Transmitter).
///  I2C_FLAG_RXNE- data register not empty (Receiver) flag.
///  I2C_FLAG_STOPF - Stop detection flag (Slave mode).
///  I2C_FLAG_ADD10 - 10-bit header sent flag (Master mode).
///  I2C_FLAG_BTF - Byte transfer finished flag.
///  I2C_FLAG_ADDR - address sent flag (Master mode) "ADSL" address matched flag (Slave mode)"ENDA".
///  I2C_FLAG_SB - Start bit flag (Master mode).
/// @return the flag is SET or RESET.
FlagStatus i2c_get_flag_status(I2C_Regs *I2Cx, uint32_t flag);

/// @brief Clears the i2c's pending flags.
///  Note:
///   - STOPF (STOP detection) is cleared by software sequence: a read operation
///     to I2C_STAR1 register (i2c_get_flag_status()) followed by a write operation
///     to I2C_CTLR1 register (i2c_cmd() to re-enable the I2C peripheral).
///   - ADD10 (10-bit header sent) is cleared by software sequence: a read
///     operation to I2C_SATR1 (i2c_get_flag_status()) followed by writing the
///     second byte of the address in DATAR register.
///   - BTF (Byte Transfer Finished) is cleared by software sequence: a read
///     operation to I2C_SATR1 register (i2c_get_flag_status()) followed by a
///     read/write to I2C_DATAR register (i2c_send_data()).
///   - ADDR (address sent) is cleared by software sequence: a read operation to
///     I2C_SATR1 register (i2c_get_flag_status()) followed by a read operation to
///     I2C_SATR2 register ((void)(i2c->SR2)).
///   - SB (Start Bit) is cleared software sequence: a read operation to I2C_STAR1
///     register (i2c_get_flag_status()) followed by a write operation to I2C_DATAR
///     register (i2c_send_data()).
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param flag specifies the flag to clear.
///  I2C_FLAG_SMBALERT - SMBus Alert flag.
///  I2C_FLAG_TIMEOUT - Timeout or Tlow error flag.
///  I2C_FLAG_PECERR - PEC error in reception flag.
///  I2C_FLAG_OVR - Overrun/Underrun flag (Slave mode).
///  I2C_FLAG_AF - Acknowledge failure flag.
///  I2C_FLAG_ARLO - Arbitration lost flag (Master mode).
///  I2C_FLAG_BERR - Bus error flag.
/// @return none
void i2c_clear_flag(I2C_Regs *I2Cx, uint32_t I2C_FLAG);

/// @brief Checks whether the specified I2C interrupt has occurred or not.
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param II2C_IT specifies the interrupt source to check.
///  I2C_IT_PECERR - PEC error in reception flag.
///  I2C_IT_OVR - Overrun/Underrun flag (Slave mode).
///  I2C_IT_AF - Acknowledge failure flag.
///  I2C_IT_ARLO - Arbitration lost flag (Master mode).
///  I2C_IT_BERR - Bus error flag.
///  I2C_IT_TXE - data register empty flag (Transmitter).
///  I2C_IT_RXNE - data register not empty (Receiver) flag.
///  I2C_IT_STOPF - Stop detection flag (Slave mode).
///  I2C_IT_ADD10 - 10-bit header sent flag (Master mode).
///  I2C_IT_BTF - Byte transfer finished flag.
///  I2C_IT_ADDR - address sent flag (Master mode) "ADSL" address matched flag (Slave mode)"ENDAD".
///  I2C_IT_SB - Start bit flag (Master mode).
/// @return SET if specified interrupt has occurred, RESET if not
ITStatus i2c_get_interrupt_status(I2C_Regs *I2Cx, uint32_t I2C_IT);

/// @brief Clears the i2c interrupt pending bits.
///  Note:
///   - STOPF (STOP detection) is cleared by software sequence: a read operation
///     to I2C_STAR1 register (i2c_get_interrupt_status()) followed by a write operation to
///     I2C_CTLR1 register (i2c_cmd() to re-enable the I2C peripheral).
///   - ADD10 (10-bit header sent) is cleared by software sequence: a read
///     operation to I2C_STAR1 (i2c_get_interrupt_status()) followed by writing the second
///     byte of the address in I2C_DATAR register.
///   - BTF (Byte Transfer Finished) is cleared by software sequence: a read
///     operation to I2C_STAR1 register (i2c_get_interrupt_status()) followed by a
///     read/write to I2C_DATAR register (i2c_send_data()).
///   - ADDR (address sent) is cleared by software sequence: a read operation to
///     I2C_STAR1 register (i2c_get_interrupt_status()) followed by a read operation to
///     I2C_STAR2 register ((void)(i2c->SR2)).
///   - SB (Start Bit) is cleared by software sequence: a read operation to
///     I2C_STAR1 register (i2c_get_interrupt_status()) followed by a write operation to
///     I2C_DATAR register (i2c_send_data()).
/// @param i2c where x can be 1 to select the I2C peripheral.
/// @param interrupt specifies the interrupt pending bit to clear.
///  I2C_IT_PECERR - PEC error in reception  interrupt.
///  I2C_IT_OVR - Overrun/Underrun interrupt (Slave mode).
///  I2C_IT_AF - Acknowledge failure interrupt.
///  I2C_IT_ARLO - Arbitration lost interrupt (Master mode).
///  I2C_IT_BERR - Bus error interrupt.
/// @return none
void i2c_clear_iterrupt_pending_bit(I2C_Regs *I2Cx, uint32_t I2C_IT);

#endif /*__CH32V00x_I2C_H */
