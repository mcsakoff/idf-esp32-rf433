#include "rf433_pulse_parser.h"
#include "rf433_parser.h"
#include "rf433_utils.h"

#include <esp_log.h>

static const char *TAG = "rf_pulse_parser";

typedef struct {
    parser_t parent;
    parser_runtime_t runtime;

    pulse_parser_config_t config;

    range_t sync_us;      // sync tick length
    range_t sync_ratio;   // second pulse width / first pulse width
    range_t bit_us;       // bit tick length
} pulse_parser_t;

/**********************************************************************************
 * Private Methods
 **********************************************************************************/

static inline bool is_sync_ratio(pulse_parser_t *p, int first, int second) {
    return p->config.inverted ?
       is_within_range(divint(first, second), &p->sync_ratio) :
       is_within_range(divint(second, first), &p->sync_ratio);
}

static inline bool parse_next_tick(pulse_parser_t *p,  rf_event_t *event) {
    int first_us = p->runtime.first_pulse.time_us;
    int second_us = p->runtime.second_pulse.time_us;

    if (first_us == 0 || second_us == 0) {
        return reset(&p->runtime, event);
    }

    if (p->runtime.captured.bits == -1) { // <-- looking for SYNC
        if (!is_sync_ratio(p, first_us, second_us)) return false;

        // sync pulse found
        int sync_width = first_us + second_us;
        // does it look like previous SYNC?
        if (is_within_range(sync_width, &p->sync_us)) {
            // just use already calculated values to save some CPU time
        } else {
            int base_pulse_width = divint(sync_width, p->config.sync_clk);
            int base_bit_width = base_pulse_width * p->config.bit_clk;
            make_range(&p->bit_us, base_bit_width, 4);
            make_range(&p->sync_us, sync_width, 1);
        }
        // start reading data bits
        start_new_code(&p->runtime);
        return false;
    }

    // check next bit tick (high + low pulses) is within base bit tick width (+- 4%)
    int bit_width = first_us + second_us;
    if (is_within_range(bit_width, &p->bit_us)) {
        // looks like a bit
    } else if (is_within_range(bit_width, &p->sync_us) && is_sync_ratio(p, first_us, second_us)) { // width is SYNC and ratio is SYNC
        // found SYNC of next code
        /*
         * NOTE: We register the code only when we get SYNC pulse of next code. In that case we drop last code
         *       that doesn't have SYNC after it. That is made intentionally. Some devices send zeros as last
         *       bits in last code of the sequence.
         */
        bool event_emitted = register_code(&p->runtime, event);
        start_new_code(&p->runtime);
        return event_emitted;
    } else { // just a noise
        return reset(&p->runtime, event);
    }

    // write bit
    p->runtime.captured.data <<= 1;
    // TODO: check pulse's widths ratio. Now just use fast but good workaround.
    if (first_us > second_us) {
        p->runtime.captured.data |= 0x1;
    }
    p->runtime.captured.bits++;  // potentially, we can capture more bits than needed due to noise (e.g. sync missed)
    if (p->runtime.captured.bits > p->config.code_bits_len) {  // data overflow
        return reset(&p->runtime, event);
    }
    return false;
}

/**********************************************************************************
 * Public Interface
 **********************************************************************************/

static bool IRAM_ATTR pulse_parser_input(parser_t *parser, const pulse_t *pulse, rf_event_t *event) {
    pulse_parser_t *p = __containerof(parser, pulse_parser_t, parent);

    switch (next_pulse(&p->runtime, pulse)) {
        case ParserProcessTick:
            return parse_next_tick(p, event);
        case ParserDoReset:
            return reset(&p->runtime, event);
        default:
            return false;
    }
}

parser_t *pulse_parser_new(const pulse_parser_config_t *config) {
    RF_CHECK(config, "configuration can't be null", NULL);

    pulse_parser_t *parser = malloc(sizeof(pulse_parser_t));
    RF_CHECK(parser, "cannot allocate memory for pulse_parser_t", NULL);

    parser->parent.input = pulse_parser_input;
    parser->config = *config;
    make_range(&parser->sync_ratio, config->sync_clk, 13); // for ratio 32 actual values can be in range 27..33

    init(&parser->runtime, (parser_runtime_config_t){
            .protocol_id = parser->config.id,
            .code_bits_len = parser->config.code_bits_len,
            .inverted = parser->config.inverted,
    });
    return &parser->parent;
}
