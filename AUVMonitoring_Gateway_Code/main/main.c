#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "esp_private/esp_clk.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"
static const char *TAG = "perfmon";
static const char *TAG2 = "MEMORY-DESCRIPTION";
static const char *TAG3 = "wifi station";
#define  ESP_WIFI_SSID "FAMILIA MEJIA"
#define  ESP_WIFI_PASS "LALA8787bebe"
#define ESP_MAXIMUM_RETRY  10
static uint64_t idle0Calls = 0;
static uint64_t idle1Calls = 0;
static int s_retry_num = 0;
#if defined(CONFIG_ESP32_DEFAULT_CPU_FREQ_240)
static const uint64_t MaxIdleCalls = 1855000;
#elif defined(CONFIG_ESP32_DEFAULT_CPU_FREQ_160)
static const uint64_t MaxIdleCalls = 1233100;
#endif
esp_err_t perfmon_start();
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .scan_method = WIFI_FAST_SCAN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();
    perfmon_start();
}

static bool idle_task_0()
{
	idle0Calls += 1;
	return false;
}

static bool idle_task_1()
{
	idle1Calls += 1;
	return false;
}
static void perfmon_task()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    wifi_config_t wifi_config;
    esp_err_t result = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    wifi_ap_record_t ap_info;
    esp_err_t result2 = esp_wifi_sta_get_ap_info(&ap_info);
    int8_t rssi;
    
    if (result == ESP_OK) {
        printf("Configuración Wi-Fi actual:\n");
        printf("SSID: %s\n", (char *)wifi_config.sta.ssid);
        printf("Contraseña: %s\n", (char *)wifi_config.sta.password);
    } else {
        printf("Error al obtener la configuración Wi-Fi: %d\n", result);
    }
    if (result2 == ESP_OK) {
        printf("Información del punto de acceso:\n");
        printf("SSID: %s\n", (char *)ap_info.ssid);
        printf("RSSI: %d\n", ap_info.rssi);
    } else {
        printf("Error al obtener la información del punto de acceso: %d\n", result);
    }
	while (1)
	{
		float idle0 = idle0Calls;
		float idle1 = idle1Calls;
		idle0Calls = 0;
		idle1Calls = 0;

		int cpu0 = 100.f -  idle0 / MaxIdleCalls * 100.f;
		int cpu1 = 100.f - idle1 / MaxIdleCalls * 100.f;

		ESP_LOGI(TAG, "Core 0 at %d%%", cpu0);
		ESP_LOGI(TAG, "Core 1 at %d%%", cpu1);
        ESP_LOGI(TAG2, "Modelo: %u", chip_info.model);
        switch (chip_info.model) {
        case 1:
            ESP_LOGI(TAG2, "CHIP_ESP32");
            break;
        case 2:
            ESP_LOGI(TAG2, "CHIP_ESP32S2");
            break;
        case 9:
            ESP_LOGI(TAG2, "CHIP_ESP32S3");
            break;
        case 5:
            ESP_LOGI(TAG2, "CHIP_ESP32C3");
            break;
        case 6:
            ESP_LOGI(TAG2, "CHIP_ESP32H2");
            break;
        case 12:
            ESP_LOGI(TAG2, "CHIP_ESP32C2");
            break;
        default:
            ESP_LOGI(TAG2, "OTRO MODELO");
            break;
    }
        ESP_LOGI(TAG2, "Número de núcleos: %u", chip_info.cores);
        ESP_LOGI(TAG2, "Caracteristicas: %lu", chip_info.features);
        ESP_LOGI(TAG2, "Revisión: %d", chip_info.revision);
        ESP_LOGI(TAG2, "Fclk: %d", esp_clk_cpu_freq());
        ESP_LOGI(TAG2, "Revisión: %d",esp_clk_xtal_freq());
		// TODO configurable delay
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
esp_err_t perfmon_start()
{
	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_0, 0));
	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_1, 1));
	// TODO calculate optimal stack size
	xTaskCreate(perfmon_task, "perfmon", 2048, NULL, 1, NULL);
	return ESP_OK;
}