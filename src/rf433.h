#pragma once

#include <freertos/FreeRTOS.h>
#include "freertos/queue.h"

extern xQueueHandle rfcode_event_queue;

#define RFCODE_START 1
#define RFCODE_STOP 0
#define RFCODE_CONTINUE 2

typedef struct {
    uint8_t action;
    uint8_t bits;
    uint32_t raw_code;
    uint16_t protocol;
} RFRawEvent;

#ifdef __cplusplus
extern "C" {
#endif

void rf433_init(void);

#ifdef __cplusplus
}
#endif
