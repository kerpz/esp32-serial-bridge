// main.c - ESP32 UART <-> TCP bridge (Telnet compatible)

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/uart.h"

static const char *TAG = "serial_tcp_bridge";

/* --- Configuration --- */
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_RTS_PIN UART_PIN_NO_CHANGE
#define UART_CTS_PIN UART_PIN_NO_CHANGE
#define UART_BAUD_RATE 115200

#define TCP_PORT 23
#define TCP_BACKLOG 2
#define BUF_SIZE 1024

static volatile int client_sock = -1;

/* ---------- Utility ---------- */

static void close_client()
{
    if (client_sock != -1)
    {
        close(client_sock);
        client_sock = -1;
        ESP_LOGI(TAG, "Client socket closed");
    }
}

/* ---------- UART Init ---------- */

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

    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM,
        BUF_SIZE * 2,
        BUF_SIZE * 2,
        0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_RTS_PIN,
        UART_CTS_PIN));

    ESP_LOGI(TAG,
             "UART ready: baud=%d tx=%d rx=%d",
             UART_BAUD_RATE,
             UART_TX_PIN,
             UART_RX_PIN);
}

/* ---------- UART -> TCP ---------- */

static void uart_to_tcp_task(void *pvParameters)
{
    uint8_t *buf = malloc(BUF_SIZE);
    if (!buf)
    {
        vTaskDelete(NULL);
        return;
    }

    while (client_sock != -1)
    {

        int len = uart_read_bytes(
            UART_NUM,
            buf,
            BUF_SIZE,
            pdMS_TO_TICKS(200));

        if (len > 0)
        {
            int sent = 0;

            while (sent < len && client_sock != -1)
            {
                int ret = send(
                    client_sock,
                    buf + sent,
                    len - sent,
                    0);

                if (ret < 0)
                {
                    close_client();
                    break;
                }

                sent += ret;
            }
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/* ---------- TCP -> UART ---------- */

static void tcp_to_uart_task(void *pvParameters)
{
    uint8_t *buf = malloc(BUF_SIZE);
    if (!buf)
    {
        vTaskDelete(NULL);
        return;
    }

    int iac_mode = 0;

    while (client_sock != -1)
    {

        int rx = recv(client_sock, buf, BUF_SIZE, 0);

        if (rx > 0)
        {

            uint8_t out[BUF_SIZE];
            int out_len = 0;

            for (int i = 0; i < rx; i++)
            {
                uint8_t c = buf[i];

                if (iac_mode)
                {
                    iac_mode = 0;
                    continue;
                }

                if (c == 0xFF)
                { // Telnet IAC
                    iac_mode = 1;
                    continue;
                }

                out[out_len++] = c;
            }

            if (out_len > 0)
            {
                uart_write_bytes(UART_NUM,
                                 (char *)out,
                                 out_len);
            }
        }
        else
        {
            close_client();
            break;
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/* ---------- TCP Server ---------- */

static void tcp_server_task(void *pvParameters)
{
    int listen_sock =
        socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Socket create failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock,
               SOL_SOCKET,
               SO_REUSEADDR,
               &opt,
               sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_sock,
             (struct sockaddr *)&addr,
             sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "Bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    listen(listen_sock, TCP_BACKLOG);

    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    while (1)
    {

        struct sockaddr_in6 source_addr;
        socklen_t addr_len = sizeof(source_addr);

        int sock = accept(listen_sock,
                          (struct sockaddr *)&source_addr,
                          &addr_len);

        if (sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (client_sock != -1)
        {
            close(sock);
            continue;
        }

        /* Non-blocking socket */
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        /* Improve latency */
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP,
                   TCP_NODELAY,
                   &flag, sizeof(flag));

        setsockopt(sock, SOL_SOCKET,
                   SO_KEEPALIVE,
                   &flag, sizeof(flag));

        client_sock = sock;

        ESP_LOGI(TAG, "Client connected");

        xTaskCreate(uart_to_tcp_task,
                    "uart2tcp",
                    4096, NULL, 5, NULL);

        xTaskCreate(tcp_to_uart_task,
                    "tcp2uart",
                    4096, NULL, 5, NULL);

        while (client_sock != -1)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        ESP_LOGI(TAG, "Waiting for next client");
    }
}

/* ---------- Main ---------- */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Wi-Fi or Ethernet must be connected here */

    init_uart();

    xTaskCreate(tcp_server_task,
                "tcp_server",
                6144,
                NULL,
                5,
                NULL);
}
