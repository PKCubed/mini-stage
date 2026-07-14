#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_http_server.h"
#include "esp_ota_ops.h"

#include <math.h>

// TLC5947 Definitions
#define PIN_NUM_MOSI 2
#define PIN_NUM_CLK  4
#define PIN_NUM_XLAT 12
#define PIN_NUM_BLANK 14

static const char *TAG = "WT32_ETH01";

spi_device_handle_t spi;
uint16_t pwm_values[24] = {0};
uint8_t tx_buf[36] = {0};
uint16_t gamma_table[256] = {0};

void init_gamma_table() {
    for (int i = 0; i < 256; i++) {
        // Gamma 2.6 is generally very smooth for LEDs
        gamma_table[i] = (uint16_t)(powf((float)i / 255.0f, 2.6f) * 4095.0f + 0.5f);
    }
}

void pack_pwm_data() {
    int bit_index = 0;
    memset(tx_buf, 0, sizeof(tx_buf));
    for (int i = 23; i >= 0; i--) {
        uint16_t val = pwm_values[i] & 0x0FFF;
        for (int b = 11; b >= 0; b--) {
            if (val & (1 << b)) {
                tx_buf[bit_index / 8] |= (1 << (7 - (bit_index % 8)));
            }
            bit_index++;
        }
    }
}

void tlc5947_update() {
    pack_pwm_data();
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 288;
    t.tx_buffer = tx_buf;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) ESP_LOGE(TAG, "SPI transmit failed");

    gpio_set_level(PIN_NUM_XLAT, 1);
    gpio_set_level(PIN_NUM_XLAT, 1);
    gpio_set_level(PIN_NUM_XLAT, 0);
}

void tlc5947_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_NUM_XLAT) | (1ULL<<PIN_NUM_BLANK);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PIN_NUM_XLAT, 0);
    gpio_set_level(PIN_NUM_BLANK, 1);

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    gpio_set_level(PIN_NUM_BLANK, 0);
}

// --- Ethernet Setup ---
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void ethernet_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = 23;
    esp32_emac_config.smi_mdio_gpio_num = 18;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = -1; 

    // Power up the PHY
    gpio_reset_pin(16);
    gpio_set_direction(16, GPIO_MODE_OUTPUT);
    gpio_set_level(16, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

// --- E1.31 Receiver ---
#define E131_PORT 5568
#define DMX_DATA_OFFSET 125

void e131_task(void *pvParameters) {
    uint8_t rx_buffer[1024];

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(E131_PORT);

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(sock);
            break;
        }
        ESP_LOGI(TAG, "E1.31 Socket bound, listening on port %d", E131_PORT);

        while (1) {
            struct sockaddr_storage source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else if (len > 125) { 
                uint8_t start_code = rx_buffer[DMX_DATA_OFFSET];
                if (start_code == 0x00) { 
                    for (int i = 0; i < 24; i++) {
                        // Ensure we don't read past the packet length
                        if (DMX_DATA_OFFSET + 1 + i < len) {
                            uint8_t dmx_val = rx_buffer[DMX_DATA_OFFSET + 1 + i];
                            pwm_values[i] = gamma_table[dmx_val];
                        } else {
                            pwm_values[i] = 0;
                        }
                    }
                    tlc5947_update();
                }
            }
        }
        if (sock != -1) {
            shutdown(sock, 0);
            close(sock);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

// --- OTA Web Server ---
const char index_html[] = 
"<!DOCTYPE html><html><head><title>WT32-ETH01 OTA</title>"
"<style>body{font-family:sans-serif; text-align:center; padding:50px; background:#1e1e1e; color:#fff;} "
"input, button{padding:10px; margin:10px; border-radius:5px; border:none;}</style></head>"
"<body><h2>WT32-ETH01 Firmware Update</h2>"
"<input type='file' id='file'><button onclick='upload()'>Upload</button>"
"<p id='status'></p>"
"<script>function upload(){"
"const f = document.getElementById('file').files[0];"
"if(!f) return;"
"document.getElementById('status').innerText = 'Uploading... Please wait.';"
"fetch('/update', {method:'POST', body:f})"
".then(res => { if(res.ok) document.getElementById('status').innerText = 'Success! Rebooting...'; else document.getElementById('status').innerText = 'Error!'; })"
".catch(err => { document.getElementById('status').innerText = 'Failed: ' + err; });"
"}</script></body></html>";

static esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t update_post_handler(httpd_req_t *req) {
    char buf[1024];
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found! Check your partition table.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting OTA to partition %s", update_partition->label);
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) continue;
            esp_ota_end(update_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        err = esp_ota_write(update_handle, (const void *)buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_end(update_handle);
            httpd_resp_send_500(req);
            return err;
        }
        remaining -= recv_len;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    httpd_resp_sendstr(req, "OK");
    ESP_LOGI(TAG, "OTA Success, restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = index_get_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_post = {
    .uri      = "/update",
    .method   = HTTP_POST,
    .handler  = update_post_handler,
    .user_ctx = NULL
};

void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);
        ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting WT32-ETH01 E1.31 Node");
    
    // Initialize NVS (Required for OTA and Ethernet)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_gamma_table();
    tlc5947_init();
    ethernet_init();
    start_webserver();
    
    xTaskCreate(e131_task, "e131_task", 4096, NULL, 5, NULL);
}
