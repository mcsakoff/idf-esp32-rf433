#include "driver/rf_receiver.h"
#include "rf433_types.h"
#include "rf433_pulse_parser.h"

#include <string.h>
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
static uint8_t s_events_mask = RF_EVENT_START | RF_EVENT_CONTINUE | RF_EVENT_STOP;

static parser_t *parsers[] = {
#ifdef CONFIG_RF_MODULE_PROTOCOL_EV1527
        NULL,
#endif
};
static int parsers_num = 0;

/*****************************************************************************
 * Task for parsing pulses from RF module
 *****************************************************************************/

static void rf_parser_task(void *arg) {
    ESP_LOGI(TAG, "start parsers task");

    pulse_t pulse;
    rf_event_t event;
    bool queue_full = false;
    for (;;) {
        if (xQueueReceive(s_pulses_queue, &pulse, portMAX_DELAY)) {
            // feed pulses to protocol parsers
            for (int n = 0; n < parsers_num; n++) {
                if (parsers[n]->input(parsers[n], &pulse, &event)) {
                    if (!(s_events_mask & BIT(event.action))) {
                        continue;
                    }
                    UBaseType_t res = xQueueSend(s_events_queue, &event, 500 / portTICK_PERIOD_MS);
                    if (res == pdFALSE) {
                        if (!queue_full) {
                            ESP_LOGE(TAG, "events queue is full");
                        }
                        queue_full = true;
                    } else {
                        queue_full = false;
                    }
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
    s_events_mask = config->events;

    ESP_LOGI(TAG, "Pulses queue: %d | Events queue: %d | Events Mask: 0x%01x",
            config->pulses_queue_size, config->events_queue_size, s_events_mask);
    return ESP_OK;
}

esp_err_t rf_driver_install(int intr_alloc_flags) {
    RF_CHECK(s_events_queue == NULL, "driver already installed", ESP_ERR_INVALID_ARG);

    // create protocol parsers
    parser_t *parser;

#ifdef CONFIG_RF_MODULE_PROTOCOL_EV1527
    parser = pulse_parser_new(&(pulse_parser_config_t) {
            .id = 0x1527,
            .sync_clk = 32,
            .bit_clk = 4,
            .code_bits_len = 24,
            .inverted = false,
    });
    if (parser == NULL) {
        ESP_LOGE(TAG, "EV1527 parser memory allocation error");
    } else {
        ESP_LOGI(TAG, "EV1527 parser created");
        parsers[parsers_num++] = parser;
    }
#endif

    if (parsers_num == 0) {
        ESP_LOGW(TAG, "no protocols parsers created");
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
