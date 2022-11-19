#pragma once

#include "rf433_types.h"

#include <esp_err.h>

#define RF_CHECK(a, str, ret_val)                                 \
    if (!(a)) {                                                   \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

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

static inline void set_range(range_t *range, int min, int max) {
    range->min = min;
    range->max = max;
}
