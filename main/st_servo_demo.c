#include "st_servo_demo.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "st_servo.h"

#define SERVO_UART_PORT UART_NUM_1
#define SERVO_BAUD_RATE 1000000
#define SERVO_TX_PIN GPIO_NUM_19
#define SERVO_RX_PIN GPIO_NUM_18
#define NUM_SERVOS 1

static const char *TAG = "st_servo_demo";
static st_servo_t g_servo;
static bool g_inited = false;

static void log_servo_feedback(st_servo_t *servo, uint8_t id)
{
    int pos = st_servo_read_pos(servo, id);
    int speed = st_servo_read_speed(servo, id);
    int load = st_servo_read_load(servo, id);
    int current = st_servo_read_current(servo, id);
    int voltage = st_servo_read_voltage(servo, id);
    int temperature = st_servo_read_temperature(servo, id);
    int moving = st_servo_read_move(servo, id);

    ESP_LOGI(TAG,
             "ID=%d pos=%d speed=%d load=%d current=%d voltage=%d temp=%d moving=%d",
             id,
             pos,
             speed,
             load,
             current,
             voltage,
             temperature,
             moving);
}

static esp_err_t servo_init_once(void)
{
    if (g_inited) {
        return ESP_OK;
    }

    st_servo_init(&g_servo, 0, 1);

    esp_err_t err = st_servo_attach_uart(&g_servo,
                                         SERVO_UART_PORT,
                                         SERVO_BAUD_RATE,
                                         SERVO_TX_PIN,
                                         SERVO_RX_PIN,
                                         512,
                                         0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "st_servo_attach_uart failed: %s", esp_err_to_name(err));
        return err;
    }

    g_inited = true;
    ESP_LOGI(TAG, "Servo UART initialized on UART%d", SERVO_UART_PORT);
    return ESP_OK;
}

void st_servo_demo_once(void)
{
    if (servo_init_once() != ESP_OK) {
        return;
    }

    int ping = st_servo_ping(&g_servo, 1);
    ESP_LOGI(TAG, "Ping servo ID=1 -> %d", ping);

    if (ping < 0) {
        return;
    }

    /* Optional service operations:
     * st_servo_calibration_offset(&g_servo, 1);   // set current position as zero point
     * st_servo_change_id(&g_servo, 1, 2);         // change servo ID 1 -> 2
     */

    int write_result = st_servo_write_pos_ex(&g_servo, 1, 2048, 1500, 50);
    ESP_LOGI(TAG, "WritePosEx result: %d", write_result);

    uint8_t ids[NUM_SERVOS] = {1};
    int16_t positions[NUM_SERVOS] = {1024};
    uint16_t speeds[NUM_SERVOS] = {3700};
    uint8_t accelerations[NUM_SERVOS] = {150};

    for (uint8_t counter = 0; counter < 5; counter++) {
        st_servo_sync_write_pos_ex(&g_servo, ids, NUM_SERVOS, positions, speeds, accelerations);
        ESP_LOGI(TAG, "SyncWritePosEx sent -> %d", positions[0]);
        for (uint8_t sample = 0; sample < 5; sample++) {
            log_servo_feedback(&g_servo, ids[0]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        positions[0] = 0;
        st_servo_sync_write_pos_ex(&g_servo, ids, NUM_SERVOS, positions, speeds, accelerations);
        ESP_LOGI(TAG, "SyncWritePosEx sent -> %d", positions[0]);
        for (uint8_t sample = 0; sample < 5; sample++) {
            log_servo_feedback(&g_servo, ids[0]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        positions[0] = 1024;
    }
}
