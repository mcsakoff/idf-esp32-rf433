#include "rf433_nec_parser.h"
#include "rf433_parser.h"
#include "rf433_utils.h"

#include <esp_log.h>

static const char *TAG = "rf_nec_parser";

typedef struct {
    parser_t parent;
    parser_runtime_t runtime;

    range_t sync_start_us;
    range_t sync_width_us;
    range_t bit_start_us;
    range_t bit_width_0_us;
    range_t bit_width_1_us;
} nec_parser_t;

/**********************************************************************************
 * Private Methods
 **********************************************************************************/

static inline bool is_sync(nec_parser_t *p, int first, int second) {
    return
        is_within_range(first, &p->sync_start_us) &&
        is_within_range(second, &p->sync_width_us);
}

static inline bool parse_next_tick(nec_parser_t *p,  rf_event_t *event) {
    int first_us, second_us, bit_width;
    first_us = p->runtime.first_pulse.time_us;
    second_us = p->runtime.second_pulse.time_us;

    if (first_us == 0 || second_us == 0) {
        return reset(&p->runtime, event);
    }
    bit_width = first_us + second_us;

    // looking for START
    if (p->runtime.captured.bits == -1) {
        if (is_sync(p, first_us, bit_width)) {
            // sync pulse found
            start_new_code(&p->runtime);
        }
        return false;
    }

    // That is very relaxed test for '0' and '1' but it works pretty well taking
    // into account huge signal drift.
    if (is_within_range(bit_width, &p->bit_width_0_us)) {
        p->runtime.captured.data <<= 1;
        p->runtime.captured.bits++;
        return false;
    }
    if (is_within_range(bit_width, &p->bit_width_1_us)) {
        p->runtime.captured.data <<= 1;
        p->runtime.captured.data |= 0x1;
        p->runtime.captured.bits++;
        return false;
    }
    if (is_sync(p, first_us, bit_width)) { // got next sync
        if (register_code(&p->runtime, event)) {
            start_new_code(&p->runtime);
            return true;
        } else {
            reset(&p->runtime, NULL);
            start_new_code(&p->runtime);
            return false;
        }
    }
//    if (p->runtime.captured.bits != p->runtime.code_bits_len && p->runtime.captured.bits > 1) {
//        ets_printf("%d + %d = %d, bit: %d\n", first_us, second_us, bit_width, p->runtime.captured.bits);
//    }
    return reset(&p->runtime, event);
}

/**********************************************************************************
 * Public Interface
 **********************************************************************************/

/*
 * @brief Consume the next pulse
 *
 * @return
 *     true if the event must be triggered
 */
static bool IRAM_ATTR nec_parser_input(parser_t *parser, const pulse_t *pulse, rf_event_t *event) {
    nec_parser_t *p = __containerof(parser, nec_parser_t, parent);

    switch (next_pulse(&p->runtime, pulse)) {
        case ParserDoReset:
            return reset(&p->runtime, event);
        case ParserProcessTick:
            return parse_next_tick(p, event);
        default:
            return false;
    }
}

/*
 * @brief Create and initialize new parser
 *
 */
parser_t *nec_parser_new() {
    nec_parser_t *parser = malloc(sizeof(nec_parser_t));
    RF_CHECK(parser, "cannot allocate memory for nec_parser_t", NULL);

    parser->parent.input = nec_parser_input;

    set_range(&parser->sync_start_us,  185, 215);
    set_range(&parser->sync_width_us,  780, 810);
    set_range(&parser->bit_start_us,    80, 120);
    set_range(&parser->bit_width_0_us, 180, 230);
    set_range(&parser->bit_width_1_us, 370, 420);

    init(&parser->runtime, (parser_runtime_config_t){
            .protocol_id = 0x00,
            .code_bits_len = 40,
            .inverted = false,
    });
    return &parser->parent;
}
