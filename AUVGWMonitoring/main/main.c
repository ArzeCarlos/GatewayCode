#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_private/esp_clk.h"
#include "esp_freertos_hooks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "sdkconfig.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "cJSON.h"
static const char *TAG = "WIFI-CONNECTION";
static const char *TAG1 = "MQTT-CONNECTION";
static const char *TAG2 = "JSON";
static const char *TAG3 = "LoRa";
static const char *TAG4 = "Performance";
static const char *TAG5 = "CHIP-INFO";
static const char *TAG6 = "WIFI-INFO";
#define  ESP_WIFI_SSID "FAMILIA MEJIA"
#define  ESP_WIFI_PASS "LALA8787bebe"
#define ESP_MAXIMUM_RETRY  10
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
uint32_t MQTT_CONNECTED = 0;
esp_timer_handle_t timer_handler;
static uint64_t idle0Calls = 0;
static uint64_t idle1Calls = 0;
struct NodePacket {
    uint8_t dest;
    uint8_t emitter;
    float value1;
    char msg[4];
};
struct NodePacket m;
int cpu0usage,cpu1usage;
char mac_string[18];
char bssid[18];
char bssidap[18];
#if defined(CONFIG_ESP32_DEFAULT_CPU_FREQ_240)
static const uint64_t MaxIdleCalls = 1855000;
#elif defined(CONFIG_ESP32_DEFAULT_CPU_FREQ_160)
static const uint64_t MaxIdleCalls = 1233100;
#endif
esp_err_t perfmon_start();
/*-------------LoRa Library.-------------*/
/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42
/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

#define CONFIG_CS_GPIO 18
#define CONFIG_RST_GPIO 14
#define CONFIG_MISO_GPIO 19
#define CONFIG_MOSI_GPIO 27
#define CONFIG_SCK_GPIO 5

static spi_device_handle_t __spi;

static int __implicit;
static long __frequency;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void 
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in  
   };

   gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_CS_GPIO, 1);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int
lora_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_CS_GPIO, 1);
   return in[1];
}

/**
 * Perform physical reset on the Lora chip
 */
void 
lora_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void 
lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void 
lora_implicit_header_mode(int size)
{
   __implicit = 1;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void 
lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void 
lora_sleep(void)
{ 
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void 
lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void 
lora_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void 
lora_set_frequency(long frequency)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void 
lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void 
lora_set_bandwidth(long sbw)
{
   int bw;

   if (sbw <= 7.8E3) bw = 0;
   else if (sbw <= 10.4E3) bw = 1;
   else if (sbw <= 15.6E3) bw = 2;
   else if (sbw <= 20.8E3) bw = 3;
   else if (sbw <= 31.25E3) bw = 4;
   else if (sbw <= 41.7E3) bw = 5;
   else if (sbw <= 62.5E3) bw = 6;
   else if (sbw <= 125E3) bw = 7;
   else if (sbw <= 250E3) bw = 8;
   else bw = 9;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void 
lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void 
lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void 
lora_set_sync_word(int sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void 
lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void 
lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int 
lora_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   /*gpio_pad_select_gpio(CONFIG_RST_GPIO);*/
   gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
   /*gpio_pad_select_gpio(CONFIG_CS_GPIO);*/
   gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);

   spi_bus_config_t bus = {
      .miso_io_num = CONFIG_MISO_GPIO,
      .mosi_io_num = CONFIG_MOSI_GPIO,
      .sclk_io_num = CONFIG_SCK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0
   };
           
   ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
      .clock_speed_hz = 9000000,
      .mode = 0,
      .spics_io_num = -1,
      .queue_size = 1,
      .flags = 0,
      .pre_cb = NULL
   };
   ret = spi_bus_add_device(VSPI_HOST, &dev, &__spi);
   assert(ret == ESP_OK);

   /*
    * Perform hardware reset.
    */
   lora_reset();

   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = lora_read_reg(REG_VERSION);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   lora_sleep();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(17);

   lora_idle();
   return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void 
lora_send_packet(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++) 
      lora_write_reg(REG_FIFO, *buf++);
   
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
   
   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);

   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param m Packet data struct
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int 
lora_receive_packet(void)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS);
   lora_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
   else len = lora_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   lora_idle();   
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > sizeof(m)) len = sizeof(m);
   char mem[len];
   for(int i=0; i<len; i++){
      mem[i]= lora_read_reg(REG_FIFO);
   } 
   memcpy(&m, mem, sizeof(m));
   printf("param1: %d, param2: %d, param3: %0.2f-%s\n", m.dest, m.emitter, m.value1,m.msg);
   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int
lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Return last packet's RSSI.
 */
int 
lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float 
lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void 
lora_close(void)
{
   lora_sleep();
//   close(__spi);  FIXME: end hardware features after lora_close
//   close(__cs);
//   close(__rst);
//   __spi = -1;
//   __cs = -1;
//   __rst = -1;
}

void 
lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", lora_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}

// MQTT Y WIFI
static void mqtt_app_start(void);
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
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG1, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG1, "MQTT_EVENT_CONNECTED");
        MQTT_CONNECTED=1;
        msg_id = esp_mqtt_client_subscribe(client, "values", 0);
        ESP_LOGI(TAG1, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "vel", 0);
        ESP_LOGI(TAG1, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG1, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED=0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG1, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG1, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG1, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        char topic_str[6];
        char velgatewayStr[6];
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        sprintf(topic_str,"%.*s", event->topic_len, event->topic);
        if(!(strcmp(topic_str,"vel"))){
            cJSON *root = cJSON_Parse((char *)event->data);
            if (root == NULL) {
            ESP_LOGE(TAG2, "Failed to parse JSON data");
            }
            else{
                ESP_LOGI(TAG2, "Deserialize.....");
                if (cJSON_GetObjectItem(root, "velgateway")) {
                    char *velgateway = cJSON_GetObjectItem(root,"velgateway")->valuestring; 
                    sprintf(velgatewayStr,"%s",velgateway);            
	            }
                cJSON_Delete(root);
                ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler,atoi(velgatewayStr)*1000000));
            } 
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG1, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG1, "Other event id:%d", event->event_id);
        break;
    }
}
esp_mqtt_client_handle_t client = NULL;
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG1, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .broker.address.uri = "mqtt://192.168.0.8",
        .broker.address.port = 1883,
        .credentials.username = "test",
        .credentials.client_id= "esp-client-1",
        .credentials.authentication.password ="12345"};
    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
void timer_callback(void *param)
{
    static bool ON;
    ON = !ON;
    ESP_LOGI(TAG2, "Serialize.....");
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "usecpu0", cpu0usage);
    cJSON_AddNumberToObject(root, "usecpu1", cpu1usage);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG5, "Modelo: %u", chip_info.model);
    switch (chip_info.model) {
        case 1:
            ESP_LOGI(TAG5, "CHIP_ESP32_D0WDQ6");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32");
            break;
        case 2:
            ESP_LOGI(TAG5, "CHIP_ESP32S2");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32S2");
            break;
        case 9:
            ESP_LOGI(TAG5, "CHIP_ESP32S3");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32S3");
            break;
        case 5:
            ESP_LOGI(TAG5, "CHIP_ESP32C3");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32C3");
            break;
        case 6:
            ESP_LOGI(TAG5, "CHIP_ESP32H2");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32H2");
            break;
        case 12:
            ESP_LOGI(TAG5, "CHIP_ESP32C2");
            cJSON_AddStringToObject(root, "model", "CHIP_ESP32C2");
            break;
        default:
            ESP_LOGI(TAG5, "OTRO MODELO");
            cJSON_AddStringToObject(root, "model", "OTRO MODELO");
            break;
    }
    ESP_LOGI(TAG5, "Número de núcleos: %u", chip_info.cores);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    ESP_LOGI(TAG5, "Caracteristicas: %lu", chip_info.features);
    cJSON_AddNumberToObject(root, "features", chip_info.features);
    ESP_LOGI(TAG5, "Revisión: %d", chip_info.revision);
    cJSON_AddNumberToObject(root, "revision", chip_info.revision);
    ESP_LOGI(TAG5, "Fclk: %d", esp_clk_cpu_freq());
    cJSON_AddNumberToObject(root, "clkcpu", esp_clk_cpu_freq()/1000000);
    ESP_LOGI(TAG5, "Revisión: %d",esp_clk_xtal_freq());
    cJSON_AddNumberToObject(root, "xtalclk", esp_clk_xtal_freq()/1000000);
    cJSON_AddNumberToObject(root, "dest", m.dest);
    cJSON_AddNumberToObject(root, "emitter", m.emitter);
    cJSON_AddNumberToObject(root, "value1", m.value1);
    cJSON_AddStringToObject(root, "msg", m.msg);
    wifi_config_t wifi_config;
    esp_err_t result = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    if (result == ESP_OK) {
        ESP_LOGI(TAG6, "SSID: %s",(char *)wifi_config.sta.ssid);
        cJSON_AddStringToObject(root, "ssid",(char *)wifi_config.sta.ssid);
        switch (wifi_config.sta.scan_method) {
            case 0:
                ESP_LOGI(TAG6, "FAST_SCAN");
                cJSON_AddStringToObject(root, "scanmethod", "FAST_SCAN");
                break;
            default:
                ESP_LOGI(TAG6, "ALL_CHANNEL_SCAN");
                cJSON_AddStringToObject(root, "scanmethod", "ALL_CHANNEL_SCAN");
                break;
        }
        ESP_LOGI(TAG6, "CHANNEL: %d",wifi_config.sta.channel);
        cJSON_AddNumberToObject(root, "channel" ,wifi_config.sta.channel);
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        sprintf(mac_string, "%02X:%02X:%02X:%02X:%02X:%02X",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG6, "MAC: %s",mac_string);
        cJSON_AddStringToObject(root, "mac", mac_string);
        sprintf(bssid, "%02X:%02X:%02X:%02X:%02X:%02X",wifi_config.sta.bssid[0], wifi_config.sta.bssid[1], 
        wifi_config.sta.bssid[2], wifi_config.sta.bssid[3], wifi_config.sta.bssid[4], wifi_config.sta.bssid[5]);
        cJSON_AddStringToObject(root, "bssid", bssid);
        switch (wifi_config.sta.sort_method) {
            case 0:
                ESP_LOGI(TAG6, "WIFI_CONNECT_AP_BY_SIGNAL");
                cJSON_AddStringToObject(root, "sortmethod", "WIFI_CONNECT_AP_BY_SIGNAL");
                break;
            default:
                ESP_LOGI(TAG6, "WIFI_CONNECT_AP_BY_SIGNAL");
                cJSON_AddStringToObject(root, "sortmethod", "WIFI_CONNECT_AP_BY_SECURITY");
                break;
        }
        ESP_LOGI(TAG6, "RSSI: %d",wifi_config.sta.threshold.rssi);
        cJSON_AddNumberToObject(root, "rssithr" ,wifi_config.sta.threshold.rssi);
        switch (wifi_config.sta.threshold.authmode) {
            case 0:
                ESP_LOGI(TAG6, "WIFI_AUTH_OPEN");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_OPEN");
                break;
            case 1:
                ESP_LOGI(TAG6, "WIFI_AUTH_WEP");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WEP");
                break;
            case 2:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA_PSK");
                break;
            case 3:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA2_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA2_PSK");
                break;
            case 4:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA_WPA2_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA_WPA2_PSK");
                break;
            case 5:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA2_ENTERPRISE");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA2_ENTERPRISE");
                break;
            case 6:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA3_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA3_PSK");
                break;
            case 7:
                ESP_LOGI(TAG6, "WIFI_AUTH_WPA2_WPA3_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WPA2_WPA3_PSK");
                break;
            case 8:
                ESP_LOGI(TAG6, "WIFI_AUTH_WAPI_PSK");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_WAPI_PSK");
                break;
            case 9:
                ESP_LOGI(TAG6, "WIFI_AUTH_OWE");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_OWE");
                break;
            case 10:
                ESP_LOGI(TAG6, "WIFI_AUTH_MAX");
                cJSON_AddStringToObject(root, "authmode", "WIFI_AUTH_MAX");
                break;
            default:
                ESP_LOGI(TAG6, "UNDEFINED");
                cJSON_AddStringToObject(root, "authmode", "UNDEFINED");
                break;
        }
    } else {
        printf("Error al obtener la configuración Wi-Fi: %d\n", result);
    }
    wifi_interface_t ifx = ESP_IF_WIFI_STA;
    uint8_t protocol;
    esp_wifi_get_protocol(ifx, &protocol);
    switch (protocol) {
            case 1:
                ESP_LOGI(TAG6, "WIFI_PROTOCOL_11B ");
                cJSON_AddStringToObject(root, "protocol", "802.11b");
                break;
            case 2:
                ESP_LOGI(TAG6, "WIFI_PROTOCOL_11G ");
                cJSON_AddStringToObject(root, "protocol", "802.11bg");
                break;
            case 4:
                ESP_LOGI(TAG6, "WIFI_PROTOCOL_11N ");
                cJSON_AddStringToObject(root, "protocol", "802.11bgn");
                break;
            case 8:
                 ESP_LOGI(TAG6, "WIFI_PROTOCOL_LR  ");
                cJSON_AddStringToObject(root, "protocol", "WIFI_PROTOCOL_LR ");
                break;
            default:
                ESP_LOGI(TAG6, "WIFI_PROTOCOL_11N ");
                cJSON_AddStringToObject(root, "protocol", "802.11bgn");
                break;
    }
    wifi_bandwidth_t bandwidth;
    esp_err_t err = esp_wifi_get_bandwidth(ifx, &bandwidth);
    if (err == ESP_OK) {
        switch (bandwidth) {
            case 1:
                ESP_LOGI(TAG6, "WIFI_BW_HT20 ");
                cJSON_AddStringToObject(root, "bandwidth", "20");
                break;
            case 2:
                ESP_LOGI(TAG6, "WIFI_BW_HT40 ");
                cJSON_AddStringToObject(root, "bandwidth", "40");
                break;
            default:
                ESP_LOGI(TAG6, "WIFI_BW_HT40 ");
                cJSON_AddStringToObject(root, "bandwidth", "40");
                break;
        }
    } else {
        printf("Error al obtener el ancho de banda Wi-Fi. Código de error: %d\n", err);
    }
    wifi_ap_record_t ap_info;
    esp_err_t result2 = esp_wifi_sta_get_ap_info(&ap_info);
    if (result2 == ESP_OK) {
        cJSON_AddNumberToObject(root, "rssireal" ,ap_info.rssi);
    } else {
        printf("Error al obtener la información del punto de acceso: %d\n", result);
    }
    char *my_json_string = cJSON_Print(root);
    ESP_LOGI(TAG2, "my_json_string\n%s",my_json_string);
    esp_mqtt_client_publish(client, "values",  my_json_string, 0, 0, 0);
    cJSON_Delete(root);
    cJSON_free(my_json_string);
    gpio_set_level(GPIO_NUM_2, ON);
}
void task_rx(void *p)
{
   int x;
   for(;;) {
      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet();
         lora_receive();
      }
      vTaskDelay(1);
   }
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
    mqtt_app_start();
    const esp_timer_create_args_t my_timer_args = {
      .callback = &timer_callback,
      .name = "My Timer"};
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 10000000));
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();
    xTaskCreatePinnedToCore(&task_rx, "task_rx", 2048, NULL, 5, NULL, 0);
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
	while (1)
	{
		float idle0 = idle0Calls;
		float idle1 = idle1Calls;
		idle0Calls = 0;
		idle1Calls = 0;
		int cpu0 = 100.f -  idle0 / MaxIdleCalls * 100.f;
		int cpu1 = 100.f - idle1 / MaxIdleCalls * 100.f;
        cpu0usage=cpu0;
        cpu1usage=cpu1;
		ESP_LOGI(TAG4, "Core 0 at %d%%", cpu0);
		ESP_LOGI(TAG4, "Core 1 at %d%%", cpu1);
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
	xTaskCreatePinnedToCore(perfmon_task, "perfmon", 2048, NULL, 5, NULL, 1);
	return ESP_OK;
}