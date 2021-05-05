#pragma once


#include <stdint.h>
#include "driver/i2c.h"


#ifdef __cplusplus
extern "C" {
#endif

extern i2c_port_t i2c_port;
extern gpio_num_t i2c_gpio_sda;
extern gpio_num_t i2c_gpio_scl;
extern uint32_t i2c_frequency;
extern int i2c_timeout;

esp_err_t i2ctools_master_driver_initialize_and_install(void);
esp_err_t i2ctools_driver_delete(void);

#ifdef __cplusplus
}
#endif
