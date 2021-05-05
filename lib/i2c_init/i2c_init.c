#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c_init.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

// iO-Fly 2 I2C defaults.
gpio_num_t i2c_gpio_sda = 26;
gpio_num_t i2c_gpio_scl = 5;
uint32_t i2c_frequency = 100000;
// The BQ27510 wants to clock-stretch up to 4 ms within transactions, which is
// 320,000 ABP bus cycles plus some headroom.
int i2c_timeout = 350000;
i2c_port_t i2c_port = I2C_NUM_0;//CONFIG_I2C_PORT;

static esp_err_t i2c_master_driver_initialize()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

esp_err_t i2ctools_master_driver_initialize_and_install(void) {
    esp_err_t res = i2c_master_driver_initialize();
    if (res == ESP_OK) {
        res = i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    }

    if (res == ESP_OK && i2c_timeout >= 0) {
        res = i2c_set_timeout(i2c_port, i2c_timeout);
    }

    return res;
}

esp_err_t i2ctools_driver_delete(void) {
    return i2c_driver_delete(i2c_port);
}
