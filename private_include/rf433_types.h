#pragma once

#include <stdint.h>
#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP8266

#define portYIELD_FROM_ISR() taskYIELD()
#define GPIO_NUM_NC          -1

#endif

typedef struct {
    int level;
    int64_t time_us;
} pulse_t;

typedef struct {
    int min;
    int max;
} range_t;

typedef struct {
    int data;
    int bits;
} code_t;


typedef struct parser_s parser_t;

struct parser_s {
    /**
     * @brief Input a pulse data to parser
     *
     * @param parser:    Handle of the parser
     * @param pulse:     Next pulse
     * @param out_event: Fill the struct if event happened
     *
     * @return
     *      true if a code parsed and event structure updated
     */
    bool (*input)(parser_t *parser, const pulse_t *pulse, rf_event_t *out_event);
};
