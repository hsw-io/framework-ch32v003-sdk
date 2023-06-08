#ifndef CH32V003_SDK_H
#define CH32V003_SDK_H

#include <system_ch32v00x.h>
#include <ch32v00x_gpio.h>
#include <stdint.h>


typedef struct {
    enum SYSCLK sysclk;
    void (*custom_nmi_handler_func)(void);
    void (*custom_hard_fault_handler_func)(void);
} Setup;

void setup(Setup setup);


#endif //CH32V003_SDK_H