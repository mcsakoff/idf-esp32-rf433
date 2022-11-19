#include "rf433_parser.h"

void init(parser_runtime_t *p, const parser_runtime_config_t config) {
    p->config = config;
    if (p->config.inverted) {
        p->first_pulse.level = 0;
        p->second_pulse.level = 1;
    } else {
        p->first_pulse.level = 1;
        p->second_pulse.level = 0;
    }
    reset(p, NULL);
}

static inline void prepare_event(parser_runtime_t *p, uint8_t action, rf_event_t *event) {
    event->protocol = p->config.protocol_id;
    event->action = action;
    event->raw_code = p->registered.data;
    event->bits = p->registered.bits;
}

/*
 * @brief Reset the parser to the state when it is waiting for initial signal
 *
 * @return status what to do next
 */
inline bool reset(parser_runtime_t *p, rf_event_t *event) {
    bool event_emitted = false;
    if (event != NULL && p->codes_num != 0) {
        prepare_event(p, RF_ACTION_STOP, event);
        event_emitted = true;
    }
    p->state = WaitingFirstPulse;
    p->first_pulse.time_us = 0;
    p->second_pulse.time_us = 0;
    p->captured.bits = -1;
    p->captured.data = 0;
    p->codes_num = 0;
    return event_emitted;
}

/*
 * @brief Consume the next pulse
 *
 * @return
 *      directive what to do next
 */
inline parser_pulse_action_t next_pulse(parser_runtime_t *p, const pulse_t *pulse) {
    if (pulse->time_us == 0) {  // that is a reset signal
        return ParserDoReset;
    }
    switch (p->state) {
        case WaitingFirstPulse:
            if (pulse->level != p->first_pulse.level) {  // not a pulse we expected
                return ParserDoReset;
            }
            p->first_pulse.time_us = pulse->time_us;
            p->state = WaitingSecondPulse;
            return ParserGetNextPulse;

        case WaitingSecondPulse:
            if (pulse->level != p->second_pulse.level) {  // not a pulse we expected
                return ParserDoReset;
            }
            p->second_pulse.time_us = pulse->time_us;
            p->state = WaitingFirstPulse;
    }
    return ParserProcessTick;
}

/*
 * @brief Register parsed code
 *
 * @return
 *    true if event prepared
 */
inline bool register_code(parser_runtime_t *p, rf_event_t *event) {
    if (p->captured.bits != p->config.code_bits_len) {
        return false; // usually that means we must reset the state
    }
    if (p->codes_num == 0) {
        p->registered = p->captured;
        prepare_event(p, RF_ACTION_START, event);
    } else if (p->captured.data != p->registered.data) {  // got different code in a sequence
        prepare_event(p, RF_ACTION_STOP, event);
        p->registered = p->captured;
        prepare_event(p, RF_ACTION_START, event);
    } else {
        prepare_event(p, RF_ACTION_CONTINUE, event);
    }
    p->codes_num++;
    return true;
}

/*
 * @brief Set the parser to the state when it is ready to read data
 *
 * @return status what to do next
 */
inline void start_new_code(parser_runtime_t *p) {
    p->captured.bits = 0;
    p->captured.data = 0;
}
