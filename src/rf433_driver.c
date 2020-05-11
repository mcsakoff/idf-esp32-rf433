#include "driver/rf_receiver.h"
#include "rf433_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>

static const char *TAG = "rf433";

#define RF_CHECK(a, str, ret_val)                                 \
    if (!(a)) {                                                   \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

static gpio_num_t s_gpio_num = GPIO_NUM_NC;
static size_t s_pulses_queue_size = 120;
static size_t s_events_queue_size = 5;
static xQueueHandle s_pulses_queue = NULL;
static xQueueHandle s_events_queue = NULL;
static UBaseType_t s_parser_task_priority = 10;

/*****************************************************************************
 * Protocol Definitions
 *****************************************************************************/

typedef struct {
    const uint16_t id;                // protocol ID

    //  +---+                           +
    //  | 1 |            31             |
    //  +   +---------------------------+
    const int sync_clk;          // sync pulse width in clock ticks = 32
    const range_t sync_ratio;      // low_pulse_width / high_pulse_width = 31, observed values - 27..31
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
    range_t sync_us;
    range_t bit_us;

    code_t captured;              // .bits == -1 means we are looking for SYNC
    code_t registered;
    int codes_num;              // number of sequential captured codes
} Protocol;

static inline void start_data(Protocol *p) { // start capturing data bits
    p->captured.bits = 0;
    p->captured.data = 0;
}

static inline void send(Protocol *p, uint8_t action) {
    rf_event_t event = {
            .action = action,
            .raw_code = p->registered.data,
            .bits = p->registered.bits,
            .protocol = p->id,
    };
    xQueueSendFromISR(s_events_queue, &event, NULL);
}

static inline void register_code(Protocol *p) {
    if (p->codes_num == 0) {
        p->registered = p->captured;
        send(p, RF_ACTION_START);
    } else if (p->captured.data != p->registered.data) {  // got different code in a sequence
        send(p, RF_ACTION_STOP);
        p->registered = p->captured;
        send(p, RF_ACTION_START);
    } else {
        send(p, RF_ACTION_CONTINUE);
    }
    p->codes_num++;
}

static inline void reset(Protocol *p) { // reset parser status to the state it waits next SYNC
    if (p->codes_num > 0) {             // send stop code event some codes captured
        send(p, RF_ACTION_STOP);
    }
    p->captured.bits = -1;
    p->captured.data = 0;
    p->codes_num = 0;
}

static DRAM_ATTR Protocol protocols[] = {
#ifdef CONFIG_RF_MODULE_PROTOCOL_EV1527
        {.id = 0x1527, .sync_clk = 32, .sync_ratio = {27, 33}, .bit_clk = 4},
#endif
};
static DRAM_ATTR int protocols_num = sizeof(protocols) / sizeof(Protocol);

/*****************************************************************************
 * Protocol Parser
 *****************************************************************************/

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
 * Task for processing pulses from RF module
 *****************************************************************************/

static void rf_parser_task(void *arg) {
    ESP_LOGI(TAG, "start parsers task");

    int high_pulse_us = 0;
    int low_pulse_us = 0;

    pulse_t pulse;
    for (;;) {
        if (xQueueReceive(s_pulses_queue, &pulse, portMAX_DELAY)) {
            if (protocols_num == 0) { // no protocols selected
                continue;
            }
            if (pulse.time_us == 0) {
                for (int n = 0; n < protocols_num; n++) {
                    reset(&protocols[n]);
                }
            } else {
                // save pulse width
                switch (pulse.level) {
                    case 1: // first pulse (HIGH)
                        high_pulse_us = pulse.time_us;
                        continue;    // <-- continue after high pulse (only half-bit received)
                    case 0: // second pulse (LOW)
                        low_pulse_us = pulse.time_us;
                        break;       // <-- process after low pulse
                    default:
                        return;
                }

                // send high and low pulses to protocol parsers
                for (int n = 0; n < protocols_num; n++) {
                    read_protocol(&protocols[n], high_pulse_us, low_pulse_us);
                }
            }
        }
    }
    vTaskDelete(NULL);
}

/*****************************************************************************
 * Interrupt for getting pulses from RF module
 *****************************************************************************/

static void IRAM_ATTR rf_isr_handler(void *arg) {
    static pulse_t prev;
    static bool queue_full = false;

    pulse_t now = {
            .time_us = esp_timer_get_time(),
            .level = gpio_get_level(s_gpio_num),  // get level of next pulse because we measure AFTER edge!
    };

    pulse_t pulse;
    if (queue_full || (now.level == prev.level)) {
        // we probably missed some interrupts; reset all parsers
        pulse.time_us = 0;
    } else {
        pulse.level = prev.level;
        pulse.time_us = now.time_us - prev.time_us;
    }

    BaseType_t hp_task_awoken;
    BaseType_t res = xQueueSendFromISR(s_pulses_queue, &pulse, &hp_task_awoken);
    if (res == pdFALSE) {
        if (!queue_full) { // report only once
            ESP_EARLY_LOGE(TAG, "pulses queue is full");
        }
        queue_full = true;
    } else {
        queue_full = false;
    }
    prev = now;

    if (hp_task_awoken) {
        portYIELD_FROM_ISR();
    }
}

/*****************************************************************************
 * External interface
 *****************************************************************************/

esp_err_t rf_get_events_handle(xQueueHandle *events) {
    RF_CHECK(events != NULL, "queue address error", ESP_ERR_INVALID_ARG);
    *events = s_events_queue;
    return ESP_OK;
}

esp_err_t rf_set_pin(gpio_num_t gpio_num) {
    RF_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number is not valid", ESP_ERR_INVALID_ARG);

    s_gpio_num = gpio_num;

    gpio_config_t io_conf = {
            .pin_bit_mask = (uint64_t) 0x1 << gpio_num,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
    };
    return gpio_config(&io_conf);
}

esp_err_t rf_config(const rf_config_t *config) {
    RF_CHECK(rf_set_pin(config->gpio_num) == ESP_OK, "set GPIO for RF driver failed", ESP_ERR_INVALID_ARG);
    if (config->events_queue_size != 0) {
        s_events_queue_size = config->events_queue_size;
    }
    if (config->pulses_queue_size != 0) {
        s_pulses_queue_size = config->pulses_queue_size;
    }
    if (config->parser_task_priority != 0) {
        s_parser_task_priority = config->parser_task_priority;
    }
    return ESP_OK;
}

esp_err_t rf_driver_install(int intr_alloc_flags) {
    RF_CHECK(s_events_queue == NULL, "driver already installed", ESP_ERR_INVALID_ARG);

    if (protocols_num == 0) {
        ESP_LOGW(TAG, "no protocols enabled in menuconfig");
    }

    // create events queue
    s_events_queue = xQueueCreate(s_events_queue_size, sizeof(rf_event_t));
    s_pulses_queue = xQueueCreate(s_pulses_queue_size, sizeof(pulse_t));

    // start parsers task
#ifdef CONFIG_RF_MODULE_TASK_PINNED_TO_CORE
    xTaskCreatePinnedToCore(rf_parser_task, "rf_parser",
            2048, NULL, s_parser_task_priority, NULL, CONFIG_RF_MODULE_TASK_PINNED_TO_CORE);
#else
    xTaskCreate(rf_parser_task, "rf_parser",
            2048, NULL, s_parser_task_priority, NULL);
#endif // CONFIG_BLINK_TASK_PINNED_TO_CORE

    // setup GPIO interrupt
    ESP_ERROR_CHECK(gpio_install_isr_service(intr_alloc_flags));
    ESP_ERROR_CHECK(gpio_isr_handler_add(s_gpio_num, rf_isr_handler, NULL));
    return ESP_OK;
}
