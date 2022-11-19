#pragma once

#include "rf433_types.h"

typedef enum {
    WaitingFirstPulse,
    WaitingSecondPulse,
} parser_state_t;

typedef enum {
    ParserGetNextPulse,
    ParserProcessTick,
    ParserDoReset,
} parser_pulse_action_t;

typedef struct {
    uint16_t protocol_id;
    int code_bits_len;
    bool inverted;
} parser_runtime_config_t;

typedef struct {
    parser_runtime_config_t config;
    parser_state_t state;

    pulse_t first_pulse;  // level show is the pulse must be high or low
    pulse_t second_pulse; // level show is the pulse must be high or low
    code_t captured;      // .bits == -1 means we are looking for SYNC
    code_t registered;
    int codes_num;        // number of sequentially captured codes
} parser_runtime_t;



void init(parser_runtime_t *p, const parser_runtime_config_t config);

parser_pulse_action_t next_pulse(parser_runtime_t *p, const pulse_t *pulse);

void start_new_code(parser_runtime_t *p);

bool register_code(parser_runtime_t *p, rf_event_t *event);

bool reset(parser_runtime_t *p, rf_event_t *event);
