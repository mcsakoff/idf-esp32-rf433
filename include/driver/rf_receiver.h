#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/gpio.h>

#define RF_ACTION_START    0
#define RF_ACTION_STOP     1
#define RF_ACTION_CONTINUE 2

#define RF_EVENT_START     BIT(RF_ACTION_START)
#define RF_EVENT_STOP      BIT(RF_ACTION_STOP)
#define RF_EVENT_CONTINUE  BIT(RF_ACTION_CONTINUE)

/**
* @brief Data struct for code event
*/
typedef struct {
    uint8_t action;
    uint8_t bits;
    uint32_t raw_code;
    uint16_t protocol;
} rf_event_t;

/**
* @brief Data struct for configuration parameters
*/
typedef struct {
    gpio_num_t gpio_num;               // RF receiver's GPIO number
    size_t events_queue_size;          // Size of events queue
    size_t pulses_queue_size;          // Size of pulses queue
    UBaseType_t parser_task_priority;  // Priority of the task that parses RF data
    uint8_t events;                    // Events to send from the driver
} rf_config_t;

/**
 * @brief Default configuration
 */
#define RF_DEFAULT_CONFIG(gpio)     \
    {                               \
        .gpio_num = gpio,           \
        .events_queue_size = 5,     \
        .pulses_queue_size = 120,   \
        .parser_task_priority = 10, \
        .events = RF_EVENT_START | RF_EVENT_CONTINUE | RF_EVENT_STOP, \
    }

/**
* @brief Configure RF driver's parameters
*
* @param config Configuration parameters
*
* @return
*     - ESP_ERR_INVALID_ARG Parameter error
*     - ESP_OK Success
*/
esp_err_t rf_config(const rf_config_t *config);

/**
* @brief Initialize RF driver
*
* @param intr_alloc_flags Flags for the RF driver interrupt handler. Pass 0 for default flags.
*                         See esp_intr_alloc.h for details.
*
* @return
*     - ESP_ERR_INVALID_ARG Parameter error
*     - ESP_OK Success
*/
esp_err_t rf_driver_install(int intr_alloc_flags);

/**
* @brief Get events queue
*
* @param events Pointer to events handle
*
* @return
*     - ESP_ERR_INVALID_ARG Parameter error
*     - ESP_OK Success
*/
esp_err_t rf_get_events_handle(xQueueHandle *events);

#ifdef __cplusplus
}
#endif
