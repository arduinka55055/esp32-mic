
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"


#define WIFI_SSID       "Police"
#define WIFI_PASS       "0323994010"
#define WIFI_CONNECTED_BIT BIT0

#define UDP_TARGET_IP   "192.168.0.208"
#define UDP_TARGET_PORT 12345

#define SAMPLE_RATE     44100
#define CHUNK           240  // 240 samples * 2 bytes = 480 bytes (safe UDP)
#define I2S_PORT        0
#define I2S_BCLK        GPIO_NUM_5
#define I2S_WS          GPIO_NUM_6
#define I2S_DIN         GPIO_NUM_4

static EventGroupHandle_t wifi_event_group;
static const char *TAG = "udp_audio";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Reconnecting...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void udp_audio_stream_task(void *param)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_TARGET_PORT),
        .sin_addr.s_addr = inet_addr(UDP_TARGET_IP)
    };

    i2s_chan_handle_t rx_handle;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 32,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = GPIO_NUM_NC,
            .din = I2S_DIN,
        },
    };

    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);

    int32_t i2s_buffer[CHUNK];
    int16_t sample_buffer[CHUNK];  // 16-bit output buffer
    size_t bytes_read;

    ESP_LOGI(TAG, "Streaming audio via UDP...");

    while (1) {
        esp_err_t res = i2s_channel_read(rx_handle, i2s_buffer, sizeof(i2s_buffer), &bytes_read, pdMS_TO_TICKS(500));
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed: %d", res);
            continue;
        }

        for (int i = 0; i < CHUNK; i++)
            sample_buffer[i] = (int16_t)(i2s_buffer[i] >> 11);  // Convert to 16-bit

        int sent = sendto(sock, sample_buffer, sizeof(sample_buffer), 0,
                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0)
            ESP_LOGE(TAG, "UDP send failed: errno %d", errno);
    }
}

void app_main()
{
    nvs_flash_init();
    wifi_init_sta();
    xTaskCreate(udp_audio_stream_task, "udp_audio_stream", 8192, NULL, 5, NULL);
}
