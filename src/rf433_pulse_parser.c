#include "rf433_pulse_parser.h"

#include <esp_log.h>


static const char *TAG = "rf_parser";

#define RF_CHECK(a, str, ret_val)                                 \
    if (!(a)) {                                                   \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

typedef enum {
    WaitingFirstPulse,
    WaitingSecondPulse,
} pulse_state_t;

typedef struct {
    parser_t parent;
    pulse_parser_config_t config;
    pulse_state_t state;

    range_t sync_us;      // sync tick length
    range_t sync_ratio;   // second pulse width / first pulse width
    range_t bit_us;       // bit tick length

    pulse_t first_pulse;  // level show is the pulse must be high or low
    pulse_t second_pulse; // level show is the pulse must be high or low
    code_t captured;      // .bits == -1 means we are looking for SYNC
    code_t registered;
    int codes_num;        // number of sequentially captured codes
} pulse_parser_t;

/**********************************************************************************
 * Private Methods
 **********************************************************************************/

static inline void start_new_code(pulse_parser_t *p) { // start capturing data bits
    p->captured.bits = 0;
    p->captured.data = 0;
}

static inline void emit_event(pulse_parser_t *p, uint8_t action, rf_event_t *event) {
    event->protocol = p->config.id;
    event->action = action;
    event->raw_code = p->registered.data;
    event->bits = p->registered.bits;
}

static bool reset(pulse_parser_t *p, rf_event_t *event) {
    bool event_emitted = false;
    if (event != NULL && p->codes_num != 0) {
        emit_event(p, RF_ACTION_STOP, event);
        event_emitted = true;
    }
    p->state = WaitingFirstPulse;
    p->first_pulse.time_us = 0;
    p->second_pulse.time_us = 0;
    p->captured.bits = -1;
    p->captured.data = 0;
    p->codes_num = 0;
    return event_emitted;
};

static inline void emit_code_event(pulse_parser_t *p, rf_event_t *event) {
    if (p->codes_num == 0) {
        p->registered = p->captured;
        emit_event(p, RF_ACTION_START, event);
    } else if (p->captured.data != p->registered.data) {  // got different code in a sequence
        emit_event(p, RF_ACTION_STOP, event);
        p->registered = p->captured;
        emit_event(p, RF_ACTION_START, event);
    } else {
        emit_event(p, RF_ACTION_CONTINUE, event);
    }
    p->codes_num++;
}

static inline bool is_sync_ratio(pulse_parser_t *p, int first, int second) {
    return p->config.inverted ?
       is_within_range(divint(first, second), &p->sync_ratio) :          //    for inverted
       is_within_range(divint(second, first), &p->sync_ratio);           //    for non-inverted
}

static inline bool parse_next_tick(pulse_parser_t *p,  rf_event_t *event) {
    int first_us = p->first_pulse.time_us;
    int second_us = p->second_pulse.time_us;

    if (first_us == 0 || second_us == 0) {
        return reset(p, event);
    }

    if (p->captured.bits == -1) { // <-- looking for SYNC
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
        start_new_code(p);
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
        emit_code_event(p, event);
        start_new_code(p);
        return true;
    } else { // just a noise
        return reset(p, event);
    }

    // write bit
    p->captured.data <<= 1;
    // TODO: check pulse's widths ration. Now just use fast but good workaround.
    if (first_us > second_us) {
        p->captured.data |= 0x1;
    }
    p->captured.bits++;            // potentially, we can capture more bits than needed due to noise (e.g. sync missed)
    if (p->captured.bits > p->config.code_bits_len) {  // data overflow
        return reset(p, event);
    }
    return false;
}

/**********************************************************************************
 * Public Interface
 **********************************************************************************/

static bool parser_input(parser_t *parser, const pulse_t *pulse, rf_event_t *event) {
    pulse_parser_t *p = __containerof(parser, pulse_parser_t, parent);

    if (pulse->time_us == 0) {                           // that is a reset signal
        return reset(p, event);
    }

    switch (p->state) {
        case WaitingFirstPulse:
            if (pulse->level != p->first_pulse.level) {  // not a pulse we expected
                return reset(p, event);
            }
            p->first_pulse.time_us = pulse->time_us;
            p->state = WaitingSecondPulse;
            return false;

        case WaitingSecondPulse:
            if (pulse->level != p->second_pulse.level) {  // not a pulse we expected
                return reset(p, event);
            }
            p->second_pulse.time_us = pulse->time_us;
            p->state = WaitingFirstPulse;
    }
    return parse_next_tick(p, event);
}

parser_t *pulse_parser_new(const pulse_parser_config_t *config) {
    RF_CHECK(config, "configuration can't be null", NULL);

    pulse_parser_t *parser = malloc(sizeof(pulse_parser_t));
    RF_CHECK(parser, "cannot allocate memory for pulse_parser_t", NULL);

    parser->parent.input = parser_input;
    parser->config = *config;
    if (parser->config.inverted) {
        parser->first_pulse.level = 0;
        parser->second_pulse.level = 1;
    } else {
        parser->first_pulse.level = 1;
        parser->second_pulse.level = 0;
    }
    make_range(&parser->sync_ratio, config->sync_clk, 13); // for ratio 32 actual values can be in range 27..33
    reset(parser, NULL);
    return &parser->parent;
}
