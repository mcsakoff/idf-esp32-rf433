#pragma once

#include "driver/rf_receiver.h"
#include "rf433_types.h"

#include <stdint.h>
#include <esp_err.h>


typedef struct {
    uint16_t id;           // protocol ID

    //  +---+                           +
    //  | 1 |            31             |
    //  +   +---------------------------+
    int sync_clk;          // sync pulse width in clock ticks (32)

    //  +---------+   +
    //  |    3    | 1 |
    //  +         +---+
    //tick_t high;         // "1" bit = {3, 1}
    //  +---+         +
    //  | 1 |    3    |
    //  +   +---------+
    //tick_t low;          // "0" bit = {1, 3}
    int bit_clk;           // bit pulse width in clock ticks (4)

    int code_bits_len;     // length of the code in bits
    bool inverted;         // if inverted, first pulse will be LOW, and second one will be HIGH
                           // also for sync first pulse will be long, and second one will be short
} pulse_parser_config_t;


/**
* @brief Creat a new parser
*
* @param config: configuration of the parser
* @return
*      Handle of the parser or NULL
*/
parser_t *pulse_parser_new(const pulse_parser_config_t *config);


/*
 * Helper functions
 */

static inline unsigned int divint(int a, int b) {  // fast int div with round
    return ((1024 * a / b) + 512) / 1024;
}

static inline bool is_within_range(int value, const range_t *range) {
    return value >= range->min && value <= range->max;
}

static inline void make_range(range_t *range, int base_value, int percent) {
    int diff = divint(base_value * percent, 100);
    range->min = base_value - diff;
    range->max = base_value + diff;
}
