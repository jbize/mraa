/*
 * Author: John <jbize@godswind.org>
 * Copyright (c) 2022 Cantada, Inc..
 *
 * SPDX-License-Identifier: MIT
 *
 * Copied from rockpi4.h
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_ROCK64_GPIO_COUNT 27
#define MRAA_ROCK64_I2C_COUNT  3
#define MRAA_ROCK64_SPI_COUNT  0
#define MRAA_ROCK64_UART_COUNT 1
#define MRAA_ROCK64_PWM_COUNT  0
#define MRAA_ROCK64_AIO_COUNT  1
#define MRAA_ROCK64_PIN_COUNT  62

mraa_board_t *
        mraa_rock64();

#ifdef __cplusplus
}
#endif
