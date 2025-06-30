#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"


#define WIFI_SSID       "Police"
#define WIFI_PASS       "0323994010"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_IPV6_READY_BIT BIT1

#define UDP_MULTICAST_GROUP "ff12::abcd"  // IPv6 multicast group
#define UDP_TARGET_PORT 12345

#define SAMPLE_RATE     44100
#define CHUNK           240  // 240 samples * 2 bytes = 480 bytes (safe UDP)
#define I2S_PORT        0
#define I2S_BCLK        GPIO_NUM_5
#define I2S_WS          GPIO_NUM_6
#define I2S_DIN         GPIO_NUM_4

static EventGroupHandle_t wifi_event_group;
static const char *TAG = "udp_audio";
static int netif_index = 0;  // Network interface index

// Function to print IPv6 address type
static const char* ipv6_addr_type_name(esp_ip6_addr_type_t type) {
    switch(type) {
        case ESP_IP6_ADDR_IS_UNKNOWN: return "Unknown";
        case ESP_IP6_ADDR_IS_GLOBAL: return "Global";
        case ESP_IP6_ADDR_IS_LINK_LOCAL: return "Link-Local";
        case ESP_IP6_ADDR_IS_SITE_LOCAL: return "Site-Local";
        case ESP_IP6_ADDR_IS_UNIQUE_LOCAL: return "Unique-Local";
        case ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6: return "IPv4-Mapped";
        default: return "Invalid";
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi Connected");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_IPV6_READY_BIT);
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconnecting...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6) {
        ip_event_got_ip6_t* event = (ip_event_got_ip6_t*) event_data;
        char addr_str[INET6_ADDRSTRLEN];
        
        // Convert IPv6 address to string
        inet_ntop(AF_INET6, &event->ip6_info.ip, addr_str, INET6_ADDRSTRLEN);
        
        // Get address type and log
        const char* addr_type = ipv6_addr_type_name(esp_netif_ip6_get_addr_type(&event->ip6_info.ip));
        ESP_LOGI(TAG, "Got IPv6: %s (%s)", addr_str, addr_type);
        
        // Store network interface index
        esp_netif_t *netif = event->esp_netif;
        netif_index = esp_netif_get_netif_impl_index(netif);
        
        xEventGroupSetBits(wifi_event_group, WIFI_IPV6_READY_BIT);
    }
}

static void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strlcpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    esp_err_t err = esp_netif_create_ip6_linklocal(sta_netif);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable IPv6: %s", esp_err_to_name(err));
    }
}

static void udp_audio_stream_task(void *param)
{
    // Wait for IPv6 address assignment
    xEventGroupWaitBits(wifi_event_group, WIFI_IPV6_READY_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Starting audio streaming...");

    // Create IPv6 socket
    int sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
    if (sock < 0) {
        ESP_LOGE(TAG, "IPv6 socket create failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Set multicast hop limit (1 = local network only)
    unsigned char hop_limit = 1;
    if (setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &hop_limit, sizeof(hop_limit))) {
        ESP_LOGE(TAG, "Failed to set hop limit: errno %d", errno);
    }

    // Configure destination address (multicast group)
    struct sockaddr_in6 dest_addr = {
        .sin6_family = AF_INET6,
        .sin6_port = htons(UDP_TARGET_PORT),
        .sin6_scope_id = netif_index
    };
    inet_pton(AF_INET6, UDP_MULTICAST_GROUP, &dest_addr.sin6_addr);

    // I2S configuration
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
        // Read from I2S
        esp_err_t res = i2s_channel_read(rx_handle, i2s_buffer, sizeof(i2s_buffer), &bytes_read, pdMS_TO_TICKS(500));
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed: %d", res);
            continue;
        }

        // Convert 32-bit samples to 16-bit
        for (int i = 0; i < CHUNK; i++) {
            sample_buffer[i] = (int16_t)(i2s_buffer[i] >> 11);
        }

        // Send via multicast
        int sent = sendto(sock, sample_buffer, sizeof(sample_buffer), 0,
                         (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            ESP_LOGE(TAG, "UDP send failed: errno %d", errno);
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}

void app_main()
{
    nvs_flash_init();
    wifi_init_sta();
    xTaskCreate(udp_audio_stream_task, "udp_audio_stream", 8192, NULL, 5, NULL);
}