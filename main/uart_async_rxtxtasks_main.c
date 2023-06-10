#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
static const int RX_BUF_SIZE = 1024;

#define PWR 2
#define TXD_PIN (GPIO_NUM_16)
#define RXD_PIN (GPIO_NUM_17)

static const char *TAG = "SIM7090G";
int earfcn, pci, rsrp, rssi, rsrq, sinr, tac, cellid, mcc, mnc;
float latitude, longitude;
char dta[256];
char cmd[64];
// int currentflag = 0;
int count = 0;
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
    gpio_set_level(PWR, 1);
}
int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int state = -10;
int flag = 0;
int success = 0;
int checking = 0;
void connect_to_gns()
{

    sendData("Connect to GNSS", "AT+CGNSINF\r");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    count += 1;
}
void wakeup()
{
    gpio_set_level(PWR, 0);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    gpio_set_level(PWR, 1);
}
void checking_cell_nbiot()
{
    sendData("--Checking cell NB-IOT ---", "AT+CENG?\r");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    count += 1;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    // sendData(TX_TASK_TAG, "AT");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    while (1)
    {

        switch (state)
        {
        case -10:
            if (flag == 1)
            {
                wakeup();
            }
            state += 1;
            break;
        case -9:
            if (flag == 1)
            {
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }

            state += 1;
            break;
        case -8:
            ESP_LOGE("-----GNSS-----", "GNSS enabled");
            sendData(TX_TASK_TAG, "AT+CGNSPWR=1\r");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            state += 1;
            break;

        case -7:
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
            connect_to_gns();
            if (success == 1)
            {
                state += 1;
            }
            else
            {
                while (success == 0)
                {
                    connect_to_gns();
                }
            }

            break;
        case -6:

            // sendData(TX_TASK_TAG, "AT+SMCONF=\"QOS\",0\r");

            sendData(TX_TASK_TAG, "AT+CGNSPWR=0\r");

            ESP_LOGE("Shut-----down", "Waiting for GNSS shut down totally.....");
            vTaskDelay(25000 / portTICK_PERIOD_MS);

            state += 1;
            break;
        case -5:
            // sendData(TX_TASK_TAG, "AT+CGNSPWR=0\r");
            // vTaskDelay(20000 / portTICK_PERIOD_MS);
            state += 1;
            break;
        case -4:
            // wakeup();
            state += 1;
            break;
        case -3:
            sendData(TX_TASK_TAG, "AT+CGNSPWR=0\r");

            vTaskDelay(10000 / portTICK_PERIOD_MS);
            state += 1;
            break;
        case -2:
            sendData(TX_TASK_TAG, "AT+SMCONF=\"URL\",\"demo.thingsboard.io\",1883\r");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            state += 1;
            break;
        case -1:

            sendData(TX_TASK_TAG, "AT+SMCONF=\"KEEPTIME\",60\r");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            state += 1;
            break;

        case 0:
            sendData(TX_TASK_TAG, "AT+SMCONF=\"USERNAME\",\"hqJFKQn2AtzKZTtTO2DN\"\r");
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            state += 1;
            break;
        case 1:
            // gpio_set_level(PWR, 1);

            // sendData(TX_TASK_TAG, "ATE0\r");

            sendData(TX_TASK_TAG, "AT+SMCONF=\"CLIENTID\",\"e80e8b4d-aafc-4e68-b3ea-ec21dda52c0b\"\r");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            state += 1;
            break;
        case 2:

            sendData(TX_TASK_TAG, "AT+CNACT=0,1\r");
            vTaskDelay(5000 / portTICK_RATE_MS);
            state += 1;
            break;
        case 3:
            checking_cell_nbiot();
            vTaskDelay(1000 / portTICK_RATE_MS);
            while (checking == 0)
            {
                checking_cell_nbiot();

                if (checking == 1)
                {
                    break;
                }
            }
            // vTaskDelay(3000 / portTICK_RATE_MS);

            break;
        case 4:
            sendData(TX_TASK_TAG, "AT+SMCONN\r");
            vTaskDelay(10000 / portTICK_RATE_MS);

            state += 1;
            break;
        case 5:
            snprintf(dta, sizeof(dta), "{\"RSRP_INDEX\":%d,\"RSRQ_INDEX\":%d,\"SINR_INDEX\":%d,\"PCI_INDEX\":%d,\"cellID_INDEX\":%d,\"LONGITUTE_INDEX\":%f,\"LATITUTE_INDEX\":%f}", rsrp, rsrq, sinr, pci, cellid, longitude, latitude);
            //
            // int len = strlen(dta);
            snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"v1/devices/me/telemetry\",%d,0,0\r", strlen(dta));
            sendData(TX_TASK_TAG, cmd);
            vTaskDelay(2000 / portTICK_RATE_MS);
            state += 1;
            break;

        case 6:
        {
            sendData(TX_TASK_TAG, dta);
            vTaskDelay(2000 / portTICK_RATE_MS);
            state += 1;
            break;
        }

        case 7:

            sendData(TX_TASK_TAG, "AT+CENG?\r");
            vTaskDelay(2000 / portTICK_RATE_MS);

            state += 1;
            break;

        case 8:

            // sendData(TX_TASK_TAG, "AT+CNACT=0,0\r");

            sendData(TX_TASK_TAG, "AT+CPOWD=0\r");
            int n = (5000 * count) % 300000;
            vTaskDelay(abs(65000 + 153000 - n) / portTICK_PERIOD_MS);
            count = 0;
            flag = 1;
            state = -10;
            break;
        default:

            break;
        }
    }
}
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    uint8_t *recieve = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    int rxBytes;
    while (1)
    {
        rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if ((state < 0))
        {

            if (rxBytes > 0)
            {
                data[rxBytes] = 0;
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                if (state == -7)
                {
                    recieve = data;

                    char *lat_start = strstr((char *)recieve, "+CGNSINF: ");
                    if (lat_start)
                    {
                        lat_start += strlen("+CGNSINF: ");
                        for (int i = 0; i < 3; i++)
                        {
                            lat_start = strchr(lat_start, ',') + 1;
                        }
                        float lat;
                        float longa;

                        lat = strtof(lat_start, NULL);

                        char *lon_start = strchr(lat_start, ',') + 1;
                        longa = strtof(lon_start, NULL);
                        if ((lat != 0) && (longa != 0))
                        {
                            latitude = lat;
                            longitude = longa;
                            state += 1;
                            success = 1;
                        }
                        else
                        {
                            success = 0;
                        }
                        ESP_LOGI(TAG, "Latitude: %f, Longitude: %f", latitude, longitude);
                    }
                }
            }
        }
        else if (state > 0)
        {
            if (rxBytes > 0)
            {
                data[rxBytes] = 0;
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                if (state == 3)
                {
                    recieve = data;
                    int cell_num;
                    char system_mode[20];
                    char *cell_info = strstr((char *)recieve, "+CENG: 1,1,");

                    sscanf(cell_info, "+CENG: 1,1,%d,%19s", &cell_num, system_mode);

                    // Get the cell info string
                    ESP_LOGI(RX_TASK_TAG, "Number cell : %d", cell_num);

                    if (cell_num > 0)
                    {
                        char *cell_info = strstr((char *)recieve, "+CENG: 0,");

                        // Extract values

                        sscanf(cell_info, "+CENG: 0,\"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\"", &earfcn, &pci, &rsrp, &rssi, &rsrq, &sinr, &tac, &cellid, &mcc, &mnc);
                        ESP_LOGI(RX_TASK_TAG, "RSRP: %d", rsrp);
                        ESP_LOGI(RX_TASK_TAG, "RSRQ: %d ", rsrq);
                        ESP_LOGI(RX_TASK_TAG, "SINR: %d", sinr);
                        ESP_LOGI(RX_TASK_TAG, "PCI : %d", pci);
                        ESP_LOGI(RX_TASK_TAG, "CellID : %d", cellid);
                        state += 1;
                        checking = 1;
                    }
                    else
                    {
                        checking = 0;
                        ESP_LOGE(RX_TASK_TAG, "No cell available");
                    }
                }

                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Add a short delay
    }
    free(data);
}

void app_main(void)
{
    init();
    initGPIO();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
