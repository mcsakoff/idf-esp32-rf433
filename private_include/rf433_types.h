#pragma once

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
