// main.c
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"

static const char *TAG = "serial_tcp_bridge";

/* --- Configuration (change as needed) --- */
#define UART_NUM UART_NUM_1
#define UART_TX_PIN (17) // change to your TX pin
#define UART_RX_PIN (16) // change to your RX pin
#define UART_RTS_PIN (UART_PIN_NO_CHANGE)
#define UART_CTS_PIN (UART_PIN_NO_CHANGE)
#define UART_BAUD_RATE (115200)

#define TCP_PORT 23 // telnet port
#define TCP_BACKLOG 1
#define BUF_SIZE 1024

static int client_sock = -1;

/* Forward declarations */
static void tcp_server_task(void *pvParameters);
static void uart_to_tcp_task(void *pvParameters);
static void tcp_to_uart_task(void *pvParameters);

/* Initialize UART */
static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN));
    ESP_LOGI(TAG, "UART initialized (num=%d, baud=%d, tx=%d, rx=%d)", UART_NUM, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
}

/* TCP server task */
static void tcp_server_task(void *pvParameters)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0)
    {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, TCP_BACKLOG) != 0)
    {
        ESP_LOGE(TAG, "Socket listen failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    while (1)
    {
        struct sockaddr_in6 source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (client_sock != -1)
        {
            ESP_LOGW(TAG, "Already connected, rejecting new client");
            close(sock);
            continue;
        }

        client_sock = sock;
        char addr_str[128];
        if (source_addr.sin6_family == AF_INET)
        {
            struct sockaddr_in *sa = (struct sockaddr_in *)&source_addr;
            inet_ntop(AF_INET, &sa->sin_addr, addr_str, sizeof(addr_str));
        }
        else
        {
            inet_ntop(AF_INET6, &source_addr.sin6_addr, addr_str, sizeof(addr_str));
        }
        ESP_LOGI(TAG, "Client connected from %s", addr_str);

        xTaskCreate(uart_to_tcp_task, "uart2tcp", 4 * 1024, NULL, tskIDLE_PRIORITY + 5, NULL);
        xTaskCreate(tcp_to_uart_task, "tcp2uart", 4 * 1024, NULL, tskIDLE_PRIORITY + 5, NULL);

        while (client_sock != -1)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        ESP_LOGI(TAG, "Client disconnected, waiting for next connection");
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

/* UART -> TCP client */
static void uart_to_tcp_task(void *pvParameters)
{
    uint8_t *buf = (uint8_t *)malloc(BUF_SIZE);
    if (!buf)
    {
        vTaskDelete(NULL);
        return;
    }

    while (client_sock != -1)
    {
        int len = uart_read_bytes(UART_NUM, buf, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            int sent = 0;
            while (sent < len)
            {
                int ret = send(client_sock, (const char *)buf + sent, len - sent, 0);
                if (ret < 0)
                {
                    close(client_sock);
                    client_sock = -1;
                    break;
                }
                sent += ret;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/* TCP client -> UART with minimal telnet IAC filter */
static void tcp_to_uart_task(void *pvParameters)
{
    uint8_t *buf = (uint8_t *)malloc(BUF_SIZE);
    if (!buf)
    {
        vTaskDelete(NULL);
        return;
    }

    int iac_mode = 0; // 0 = normal, 1 = inside IAC

    while (client_sock != -1)
    {
        int rx = recv(client_sock, (char *)buf, BUF_SIZE, 0);
        if (rx > 0)
        {
            for (int i = 0; i < rx; i++)
            {
                uint8_t c = buf[i];
                if (iac_mode)
                {
                    iac_mode = 0; // skip this byte
                    continue;
                }
                else if (c == 0xFF)
                {
                    iac_mode = 1; // start IAC sequence
                    continue;
                }
                else
                {
                    uart_write_bytes(UART_NUM, (const char *)&c, 1);
                }
            }
        }
        else if (rx == 0)
        {
            close(client_sock);
            client_sock = -1;
            break;
        }
        else
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            close(client_sock);
            client_sock = -1;
            break;
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/* Main app */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // NOTE: You must connect Wi-Fi/ethernet separately

    init_uart();

    xTaskCreate(tcp_server_task, "tcp_server", 6 * 1024, NULL, tskIDLE_PRIORITY + 5, NULL);
}
