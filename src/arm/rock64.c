/*
 * Author: Brian <brian@vamrs.com>
 * Copyright (c) 2019 Vamrs Corporation.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/rock64.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
/*
* "Pine64 Rock64" is the model name on stock 5.x kernels
*/
#define PLATFORM_NAME_ROCK64 "Rock64"
#define MAX_SIZE 64

const char* rock64_serialdev[MRAA_ROCK64_UART_COUNT] = { "/dev/ttyS2" };

void
mraa_rock64_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
{
    va_list arg_ptr;
    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);

    if( pincapabilities_t.gpio == 1 ) {
        pininfo->gpio.gpio_chip = sysfs_pin / 32;
        pininfo->gpio.gpio_line = sysfs_pin % 32;
    }

    pininfo->capabilities = pincapabilities_t;

    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

mraa_board_t*
mraa_rock64()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_ROCK64_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ROCK64)) {
            b->platform_name = PLATFORM_NAME_ROCK64;
            b->uart_dev[0].device_path = (char*) rock64_serialdev[0];
            b->uart_dev[1].device_path = (char*) rock64_serialdev[1];
        }
    }

    // UART
    b->uart_dev_count = MRAA_ROCK64_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_ROCK64, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_ROCK64_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 1;
        b->i2c_bus[1].bus_id = 4;
    }

    // SPI
    b->spi_bus_count = MRAA_ROCK64_SPI_COUNT;
    b->def_spi_bus = 0;
//    b->spi_bus[0].bus_id = 32766;
//    b->spi_bus[1].bus_id = 32765;

    b->pwm_dev_count = MRAA_ROCK64_PWM_COUNT;
//    b->pwm_default_period = 500;
//    b->pwm_max_period = 2147483;
//    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[11].pwm.parent_id = 0;
    b->pins[11].pwm.mux_total = 0;
    b->pins[11].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 1;
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;

    b->aio_count = MRAA_ROCK64_AIO_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 26;
    b->aio_non_seq = 1;
    b->chardev_capable = 1;

//    typedef struct {
//        /*@{*/
//        mraa_boolean_t valid:1;     /**< Is the pin valid at all */
//        mraa_boolean_t gpio:1;      /**< Is the pin gpio capable */
//        mraa_boolean_t pwm:1;       /**< Is the pin pwm capable */
//        mraa_boolean_t fast_gpio:1; /**< Is the pin fast gpio capable */
//        mraa_boolean_t spi:1;       /**< Is the pin spi capable */
//        mraa_boolean_t i2c:1;       /**< Is the pin i2c capable */
//        mraa_boolean_t aio:1;       /**< Is the pin analog input capable */
//        mraa_boolean_t uart:1;       /**< Is the pin uart capable */
//        /*@}*/
//    } mraa_pincapabilities_t;

//    https://github.com/Leapo/Rock64-R64.GPIO/wiki/GPIO-Modes
//    http://synfare.com/599N105E/hwdocs/rock64/index.html

    mraa_rock64_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_rock64_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rock64_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rock64_pininfo(b, 3,   89, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GPIO2_D1 I2C0_SDA NET_Speed");
    mraa_rock64_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rock64_pininfo(b, 5,   88, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GPIO2_D0 I2C0_SCL NET_Link");
    mraa_rock64_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 7,   60, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GPIO1_D4 CLK32OUT_M1");
    mraa_rock64_pininfo(b, 8,   64, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GPIO2_A0 UART2_TX");
    mraa_rock64_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 10,  65, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GPIO2_A1 UART2_RX");
    mraa_rock64_pininfo(b, 11,  -1, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "NC (66 GPIO2_A2 IR_RX)");
    mraa_rock64_pininfo(b, 12,  67, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO2_A3 (input only?)");
    mraa_rock64_pininfo(b, 13,  -1, (mraa_pincapabilities_t){1,0,1,0,0,0,0,0}, "GPIO0_A0 vbus-drv USB3 power control (U2502 Enable)");
    mraa_rock64_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 15, 100, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GPIO3_A4 (uart1_tx)");
    mraa_rock64_pininfo(b, 16, 101, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A5");
    mraa_rock64_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rock64_pininfo(b, 18, 102, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GPIO3_A6 (uart1_rx)");
    mraa_rock64_pininfo(b, 19,  97, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GPIO3_A1 SPI_TXD_M2");
    mraa_rock64_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 21,  98, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GPIO3_A2 SPI_RXD_M2");
    mraa_rock64_pininfo(b, 22, 103, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A7");
    mraa_rock64_pininfo(b, 23,  96, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GPIO3_A0 SPI_CLK_M2");
    mraa_rock64_pininfo(b, 24, 104, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GPIO3_B0 SPI_CSN0_M2");
    mraa_rock64_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 26,  76, (mraa_pincapabilities_t){1,1,0,0,0,0,1,0}, "GPIO2_B4 (SPI_CSN1_M0)");
    mraa_rock64_pininfo(b, 27,  68, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GPIO2_A4 I2C1_SDA_PMIC");
    mraa_rock64_pininfo(b, 28,  69, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GPIO2_A5 I2C1_SCL_PMIC");
    mraa_rock64_pininfo(b, 29,  -1, (mraa_pincapabilities_t){1,0,0,0,1,1,0,0}, "(70 GPIO2_A6 PMIC_INT)");
    mraa_rock64_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 31,  -1, (mraa_pincapabilities_t){1,0,0,0,1,1,0,0}, "(2 GPIO0_A2 USB20_HOST_DRV, USB2 power control)");
    mraa_rock64_pininfo(b, 32,  38, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A6 SDMMC0_CLK");
    mraa_rock64_pininfo(b, 33,  32, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GPIO1_A0 SDMMC0_D0");
    mraa_rock64_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 35,  33, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A1 SDMMC0_D1");
    mraa_rock64_pininfo(b, 36,  37, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A5 SDMMC0_DET");
    mraa_rock64_pininfo(b, 37,  34, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A2 SDMMC0_D2/JTAG_TCK");
    mraa_rock64_pininfo(b, 38,  36, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A4 SDMMC0_CMD");
    mraa_rock64_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rock64_pininfo(b, 40,  35, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A3 SDMMC0_D3/JTAG_TMS DET");

    // TODO:  Add second header (Pi-5+) here:

    return b;
}
