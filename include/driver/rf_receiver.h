#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/gpio.h>

#define RF_ACTION_STOP     0
#define RF_ACTION_START    1
#define RF_ACTION_CONTINUE 2

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
