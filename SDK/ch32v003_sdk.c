#include "ch32v003_sdk.h"

void setup(Setup setup) {
    set_sys_clock(setup.sysclk);
    // override nmi and hard_fault handlers
    // if (systemSetup.custom_nmi_handler_func != NULL) {
    //     nmi_handler_func = systemSetup.custom_hard_fault_handler_func;
    // }
    // if (systemSetup.custom_hard_fault_handler_func != NULL) {
    //     hard_fault_handler_func = systemSetup.custom_hard_fault_handler_func;
    // }
    for(size_t i=0; i < setup.gpios.len; i += 1) {
        gpio_init(setup.gpios.pins_init[i]);
    }
}