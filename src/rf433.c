#include "rf433.h"

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>

#define RF433_GPIO CONFIG_RF433_RECEIVER_GPIO

xQueueHandle rfcode_event_queue = NULL;

/*****************************************************************************
 * Protocol Definitions
 *****************************************************************************/

typedef struct {
    int min;
    int max;
} Range;

typedef struct {
    int data;
    int bits;
} Code;

//typedef struct {
//    int high;
//    int low;
//} Pulses;

typedef struct {
    const uint16_t id;                // protocol ID

    //  +---+                           +
    //  | 1 |            31             |
    //  +   +---------------------------+
    const int sync_clk;          // sync pulse width in clock ticks = 32
    const Range sync_ratio;      // low_pulse_width / high_pulse_width = 31, observed values - 27..31
    //  +---------+   +
    //  |    3    | 1 |
    //  +         +---+
    //const Pulses high;         // "1" bit = {3, 1}
    //  +---+         +
    //  | 1 |    3    |
    //  +   +---------+
    //const Pulses low;          // "0" bit = {1, 3}
    const int bit_clk;           // bit pulse width in clock ticks = 4

    // protocol parameters calculated in runtime
    Range sync_us;
    Range bit_us;

    Code captured;              // .bits == -1 means we are looking for SYNC
    Code registered;
    int codes_num;              // number of sequential captured codes
} Protocol;

static inline void start_data(Protocol *p) { // start capturing data bits
    p->captured.bits = 0;
    p->captured.data = 0;
}

static inline void send(Protocol *p, uint8_t action) {
    RFRawEvent event = {
            .action = action,
            .raw_code = p->registered.data,
            .bits = p->registered.bits,
            .protocol = p->id,
    };
    xQueueSendFromISR(rfcode_event_queue, &event, NULL);
}

static inline void register_code(Protocol *p) {
    if (p->codes_num == 0) {
        p->registered = p->captured;
        send(p, RFCODE_START);
    } else if (p->captured.data != p->registered.data) {  // got different code in a sequence
        send(p, RFCODE_STOP);
        p->registered = p->captured;
        send(p, RFCODE_START);
    } else {
        send(p, RFCODE_CONTINUE);
    }
    p->codes_num++;
}

static inline void reset(Protocol *p) { // reset parser status to the state it waits next SYNC
    if (p->codes_num > 0) {             // send stop code event some codes captured
        send(p, RFCODE_STOP);
    }
    p->captured.bits = -1;
    p->captured.data = 0;
    p->codes_num = 0;
}

DRAM_ATTR static Protocol protocols[] = {
#ifdef CONFIG_RF433_PROTOCOL_EV1527
        {.id = 0x1527, .sync_clk = 32, .sync_ratio = {27, 33}, .bit_clk = 4},
#endif
};
DRAM_ATTR static int protocols_num = sizeof(protocols) / sizeof(Protocol);

/*****************************************************************************
 * Protocol Parser
 *****************************************************************************/

static inline unsigned int divint(int a, int b) {  // fast int div with round
    return ((1024 * a / b) + 512) / 1024;
}

static inline bool is_within_range(int value, const Range *range) {
    return value >= range->min && value <= range->max;
}

static inline void make_range(Range *range, int base_value, int percent) {
    int diff = divint(base_value * percent, 100);
    range->min = base_value - diff;
    range->max = base_value + diff;
}

static inline void read_protocol(Protocol *p, int high_us, int low_us) {
    if (high_us == 0 || low_us == 0) { // just a noise
        reset(p);
        return;
    }
    if (p->captured.bits == -1) { // <-- looking for SYNC
        if (!is_within_range(divint(low_us, high_us), &p->sync_ratio)) return;
        // sync pulse found

        int sync_width = high_us + low_us;
        // does it look like previous SYNC?
        if (is_within_range(sync_width, &p->sync_us)) {
            // just use already calculated spec
        } else {
            int base_pulse_width = divint(sync_width, p->sync_clk);
            int base_bit_width = base_pulse_width * p->bit_clk;
            make_range(&p->bit_us, base_bit_width, 4);
            make_range(&p->sync_us, sync_width, 1);
        }
        // start reading data bits
        start_data(p);
        return;
    }

    // check next bit width is within base bit pulse width (+- 4%)
    int bit_width = high_us + low_us;
    if (is_within_range(bit_width, &p->bit_us)) {
        // looks like bit
    } else if (is_within_range(bit_width, &p->sync_us)                      // width is SYNC
               && is_within_range(divint(low_us, high_us), &p->sync_ratio)) {  // ratio is SYNC
        // found SYNC of next code

        /*
         * NOTE: we register code only when we get SYNC pulse of next code. in that case we drop last code
         *       that doesn't have SYNC after it. that is made intentionally. some devices send
         *       broken data in last code of the sequence!
         */
        register_code(p);

        // start reading data bits for next code
        start_data(p);
        return;
    } else { // just a noise
        reset(p);
        return;
    }

    // write bit
    p->captured.data <<= 1;
    if (high_us > low_us) {  // TODO: check pulse's widths
        p->captured.data |= 0x1;
    }
    p->captured.bits++;
    if (p->captured.bits == 32) {  // data overflow
        reset(p);
    }
}

/*****************************************************************************
 * RF433 module protocol handler
 *****************************************************************************/

typedef struct {
    int level;
    int time_us;
} Event;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    static Event prev;
    static int high_pulse_us;
    static int low_pulse_us;

    Event now = {
            .time_us = esp_timer_get_time(),
            .level = gpio_get_level((gpio_num_t) RF433_GPIO),  // get level of next pulse because we measure AFTER edge!
    };
    Event pulse = {
            .time_us = now.time_us - prev.time_us,
            .level = prev.level,
    };
    // sanity check
    if (now.level == prev.level) { // we missed some interrupts probably because of noise
        // reset all parsers
        for (int n = 0; n < protocols_num; n++) {
            reset(&protocols[n]);
        }
        prev = now;
        return;
    }
    prev = now;

    // save pulse width
    switch (pulse.level) {
        case 1: // first pulse (HIGH)
            high_pulse_us = pulse.time_us;
            return;                            // <-- exit after high pulse (only half-bit received)
        case 0: // second pulse (LOW)
            low_pulse_us = pulse.time_us;
            break;                             // <-- continue after low pulse
        default:
            return;
    }

    // send high and low pulses to protocol parsers
    for (int n = 0; n < protocols_num; n++) {
        read_protocol(&protocols[n], high_pulse_us, low_pulse_us);
    }
}

void rf433_init(void) {
    rfcode_event_queue = xQueueCreate(32, sizeof(RFRawEvent));

    // setup GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (uint32_t) 0x1 << RF433_GPIO;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // setup GPIO interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RF433_GPIO, gpio_isr_handler, NULL);
}
