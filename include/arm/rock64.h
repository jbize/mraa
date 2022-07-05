/*
 * Author: Brian <brian@vamrs.com>
 * Copyright (c) 2019 Vamrs Corporation.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_ROCK64_GPIO_COUNT 27
#define MRAA_ROCK64_I2C_COUNT  2
#define MRAA_ROCK64_SPI_COUNT  0
#define MRAA_ROCK64_UART_COUNT 1
#define MRAA_ROCK64_PWM_COUNT  0
#define MRAA_ROCK64_AIO_COUNT  1
#define MRAA_ROCK64_PIN_COUNT  40

mraa_board_t *
        mraa_rock64();

#ifdef __cplusplus
}
#endif
