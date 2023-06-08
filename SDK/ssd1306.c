#include <ch32v00x_i2c.h>


void ssd1306_setup(void) {
    i2c_init((I2C_Bus) {
        .speed = 100000,
    });
}

#define TIMEOUT 100000
#define SSD1306_ADDR 0x3C

uint8_t ssd1306_send(I2C_Bus *bus, uint8_t addr, uint8_t *data, uint8_t size) {
    while(i2c_get_flag_status(bus->i2c, I2C_FLAG_BUSY));
    i2c_generate_start(bus->i2c, ENABLE);
    while(!i2c_check_event(bus->i2c, I2C_EVENT_MASTER_MODE_SELECT));
    i2c_send_7bit_address(bus->i2c, SSD1306_ADDR, I2C_DIRECTION_Transmitter);
    while(!i2c_check_event(bus->i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    for(uint8_t i=0; i < size; i+=1) {
        while(!i2c_check_event(bus->i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
        i2c_send_data(bus->i2c, data[i]);
    }
    while(!i2c_check_event(bus->i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}