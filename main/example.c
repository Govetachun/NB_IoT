/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define PWR 2
#define TXD_PIN (GPIO_NUM_16)
#define RXD_PIN (GPIO_NUM_17)

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,

    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void initGPIO(void)
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 1 << PWR;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    // gpio_set_level(PWR, 0);
}
int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int state = -1;
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    // sendData(TX_TASK_TAG, "AT");

    while (1)
    {
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // // gpio_set_level(PWR, 1);
        // // for (int j = 0; j < 6; j++)
        // // {
        // //     sendData(TX_TASK_TAG, "AT\r");
        // //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // // }
        // sendData(TX_TASK_TAG, "AT+CPOWD=0\r");
        // // vTaskDelay(3000 / portTICK_PERIOD_MS);
        // gpio_set_level(PWR, 0);
        switch (state)
        {
        case -1:
            state += 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            break;
        case 0:

            state += 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        case 1:
            gpio_set_level(PWR, 1);

            // sendData(TX_TASK_TAG, "AT\r");
            // sendData(TX_TASK_TAG, "ATE0\r");
            state += 1;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            break;
        case 6:
            state = -1;
            sendData(TX_TASK_TAG, "AT+CPOWD=0\r");

            vTaskDelay(3000 / portTICK_PERIOD_MS);

            break;
        default:
            // gpio_set_level(PWR, 1);

            sendData(TX_TASK_TAG, "AT\r");
            state += 1;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            break;
        }
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        if (state == -1)
        {
            gpio_set_level(PWR, 0);
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    initGPIO();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
