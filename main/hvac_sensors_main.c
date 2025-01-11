/*
    HVAC Sensors
    This is for all the sensors as part of the HVAC control system.

    based on WiFi station and the i2c master Examples
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   There are 3 sensor modules for the HVAC system
    1. Intake:  reads the incoming air temperature and the pressure drop
                accross the intake filter.
    2. Supply   reads the temperature and humidity of the air feed.
    3. Extract  reads the air temperature, humidity and CO2 content
                of the air extracted from the house,
                as well as the pressure drop accross the extract filter.
    NOTE: The exhaust plenum does not have any sensors fitted.

    Outside -> Intake -> Heat Exchanger -> Supply  -> house

      House -> Extact -> Heat Exchanger -> Exhaust -> outside
*/


#define FIRMWARE_VER_MAJ		0
#define FIRMWARE_VER_MIN		2

// select only 1 of the following
#define HVAC_SENSOR_INTAKE      1
// #define HVAC_SENSOR_SUPPLY      2
// #define HVAC_SENSOR_EXTRACT     3

#if (HVAC_SENSOR_INTAKE > 0)
#define SENSOR_TEXT             "Intake"
#define USES_BMP3               1
#define USES_BME2               0
#define USES_MHZ                0
#define USES_AHT                0
#define USES_LCD_SMS1706        1
#endif
#if (HVAC_SENSOR_SUPPLY > 0)
#define SENSOR_TEXT             "Supply"
#define USES_BMP3               0
#define USES_BME2               0
#define USES_MHZ                0
#define USES_AHT                1
#define USES_LCD_SMS1706        1
#endif
#if (HVAC_SENSOR_EXTRACT > 0)
#define SENSOR_TEXT             "Extract"
#define USES_BMP3               0
#define USES_BME2               1
#define USES_MHZ                1
#define USES_AHT                0
#define USES_LCD_SMS1706        1
#endif


#include <string.h>
#include <sys/param.h>      // needed for MIN() macro

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "mqtt_client.h"
// #include "components/mdns.h"        // idf.py add-dependency espressif/mdns
#include "esp_ota_ops.h"
#include "driver/i2c.h"      // old driver, migrate to i2c_master.h
#include "driver/i2c_master.h"

#include "json.h"
#include "mqtt_topics.h"
#if (USES_BMP3 > 0)
#include "bmp3.h"
#endif
#if (USES_LCD_SMS1706 > 0)
#include "lcd_sms1706.h"
#endif
#if (USES_BME2 > 0)
#include "bme280.h"
#endif
#if (USES_MHZ > 0)
#include "MHZ.h"
#include "driver/uart.h"
#endif
#if (USES_AHT > 0)
#include "AHTxx.h"
#endif


#define USE_MQTT                (1)     // enable mqtt client
#define USE_MDNS                (0)


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif


#define SSID_LEN				32
#define PSK_LEN					64
#define HOSTNAME_LEN			(15)						// 63 chars max, RFC1034 sect 3.1. 32 chars max for station name.
#define MAX_AP_LIST_ITEMS		(9)

#if (USE_MQTT > 0)
#define MQTT_SITENAME_BYTES		(30)
#define MQTT_HOSTNAME_BYTES		(50)
#define CONFIG_BROKER_URI       "mqtt://homeassistant.local:1883"        // local home assistant broker
#define CONFIG_BROKER_BIN_SIZE_TO_SEND  (250)
#endif

#define I2C_MASTER_SCL_IO           19 //CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18 //CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BMP390_CNTRL_MODE_MASK              0x30        /*!< contrl register value mask for mode bits */
#define BMP390_CNTRL_MODE_NORMAL            0x30        /*!< contrl register value for normal mode */
#define BMP390_CNTRL_PRESS_EN               0x01        /*!< contrl register value to enable pressure reading */
#define BMP390_CNTRL_TEMP_EN                0x02        /*!< contrl register value to enable temperature reading */

#define EX_UART_NUM                 UART_NUM_1
#define BUF_SIZE (1024)
// #define RD_BUF_SIZE (BUF_SIZE)


#if (USES_MHZ > 0)
static QueueHandle_t uart0_queue;
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "HVAC sensors";

static int s_retry_num      = 0;

uint8_t		u8_conn_status;									// used to keep track of the connection status

char		hostname[HOSTNAME_LEN]	= {"HVAC sensors"};	    // 63 chars max, RFC1034 section 3.1. 32 chars max for station name.
char		strSSID[SSID_LEN];
char		strPassword[PSK_LEN];
uint16_t	num_APs;										// the number of APs found by the station.
uint8_t		u8_scan_for_APs_f;

static wifi_scan_config_t	scan_config;

struct ap_list_str
{
	char	ssid[SSID_LEN];
	uint8_t	rssi;
} AP_list[MAX_AP_LIST_ITEMS];								// struct for the AP information

#if (USE_MQTT != 0)
esp_mqtt_client_handle_t	mqtt_handle;
static const char *MQTT_TAG = "MQTT_CLIENT";

struct mqtt_str
{
	char devicename[20];                // unique device name for this sensor module
	char hostname[MQTT_HOSTNAME_BYTES];
	char username[MQTT_SITENAME_BYTES];
	char password[MQTT_SITENAME_BYTES];
    char sensor_topic[15];              // the topic to publish sensor readings to
	uint8_t enabled;
	uint16_t Port;                      // mqtt port, usually 1883 or 8883
	uint16_t interval;                  // seconds between each sensor reading update
	uint32_t id;
	uint16_t filter_threshold;          // max pressure drop accross the filter in Pascals, threshold of a blocked filter.
	float filter_offset;                // correction value so filter reads ~zero when sensors in static air.
} mqtt;

char    json_payload[CONFIG_BROKER_BIN_SIZE_TO_SEND];
#endif

i2c_master_dev_handle_t     i2c_dev;
#if (USES_BMP3 > 0)
struct bmp3_dev     bmp390_dev[2];      // only 2 addresses available
#endif
#if (USES_BME2 > 0)
struct bme280_dev   bme280_sensor[2];      // only 2 addresses available
struct bme280_data  bme280_sensor_data[2];
#endif
#if (USES_LCD_SMS1706 > 0)
struct display      disp_content;
#endif


static esp_err_t sensor_register_read(uint8_t sensor, uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t sensor_register_write_byte(uint8_t sensor, uint8_t reg_addr, uint8_t data);
#if (USES_BMP3 > 0)
static void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev);
static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct bmp3_calib_data *calib_data);
static float BMP390_compensate_pressure(uint32_t uncomp_press, struct bmp3_calib_data *calib_data);
#endif
#if (USE_MQTT != 0)
static void     publish_2_mqtt(esp_mqtt_client_handle_t client, char* topic, char* payload);
static void		mqtt_app_start(void);
void            fv_sensor_write_json(float temp, float delta, float absolute);
void            fv_sensor_write_config(uint8_t element_num);
void			fv_mqtt_config_form(char *webpage);
#endif
void			fv_wifi_config_form(char *webpage);
#if (USE_MDNS != 0)
void			mdns_setup();
esp_err_t		resolve_mdns_host(const char * host_name);
#endif
void			fv_main_scan_for_APs(void);


/**
 * @brief Read a sequence of bytes from the sensor registers
 */
static esp_err_t sensor_register_read(uint8_t sensor, uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t   err;
    uint8_t     addr;
#if (USES_BMP3 > 0)
    addr                    = BMP3_ADDR_I2C_PRIM;
#endif
#if (USES_BME2 > 0)
    addr                    = BME280_I2C_ADDR_PRIM;
#endif

    if (sensor == 2)
    {
#if (USES_BMP3 > 0)
        addr                = BMP3_ADDR_I2C_SEC;
#endif
#if (USES_BME2 > 0)
        addr                = BME280_I2C_ADDR_SEC;
#endif
    }

    err         = i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    // err         = i2c_master_transmit_receive(i2c_dev, &wr_buf[0], 1, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        switch (err)
        {
        case    ESP_FAIL:
            printf("i2c fail\n");
            break;

        case    ESP_ERR_INVALID_STATE:
            printf("i2c not in master mode\n");
            break;

        case    ESP_ERR_TIMEOUT:
            printf("i2c timeout\n");
            break;

        default:
            printf("i2c read error: %d\n", err);
            break;
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    return err;
}


/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t sensor_register_write_byte(uint8_t sensor, uint8_t reg_addr, uint8_t data)
{
    int         ret;
    uint8_t     addr;
#if (USES_BMP3 > 0)
    addr                    = BMP3_ADDR_I2C_PRIM;
#endif
#if (USES_BME2 > 0)
    addr                    = BME280_I2C_ADDR_PRIM;
#endif
    uint8_t write_buf[2]    = {reg_addr, data};

    if (sensor == 2)
    {
#if (USES_BMP3 > 0)
        addr                = BMP3_ADDR_I2C_SEC;
#endif
#if (USES_BME2 > 0)
        addr                = BME280_I2C_ADDR_SEC;
#endif
    }

    ret                     = i2c_master_write_to_device(I2C_MASTER_NUM,
                                                        addr,
                                                        write_buf,
                                                        sizeof(write_buf),
                                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}


#if (USES_BMP3 > 0)
/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it then compensates it and store in the device structure.
 */
static int8_t get_calib_data(uint8_t sensor, struct bmp3_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr        = BMP3_REG_CALIB_DATA;

    /* Array to store calibration data */
    uint8_t calib_data[BMP3_LEN_CALIB_DATA]     = { 0 };

    /* Read the calibration data from the sensor */
    rslt                    = sensor_register_read(sensor, reg_addr, calib_data, BMP3_LEN_CALIB_DATA);

    /* Parse calibration data and store it in device structure */
    parse_calib_data(calib_data, dev);

    return rslt;
}


/*!
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev)
{
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data *reg_calib_data              = &dev->calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data *quantized_calib_data  = &dev->calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);
    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);
    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);
    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);
    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);
    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);
    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);
    reg_calib_data->par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);
}


static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct bmp3_calib_data *calib_data)
{
    float partial_data1;
    float partial_data2;

    partial_data1           = (float)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
    partial_data2           = (float)(partial_data1 * calib_data->quantized_calib_data.par_t2);
    // store in calib data for pressure calc
    calib_data->quantized_calib_data.t_lin       = partial_data2 + (partial_data1 * partial_data1) * calib_data->quantized_calib_data.par_t3;

    return calib_data->quantized_calib_data.t_lin;
} // end of BMP390_compensate_temperature ------------------


static float BMP390_compensate_pressure(uint32_t uncomp_press, struct bmp3_calib_data *calib_data)
{
    float comp_press;
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1           = (calib_data->quantized_calib_data.par_p6 * calib_data->quantized_calib_data.t_lin);
    partial_data2           = (calib_data->quantized_calib_data.par_p7 * (calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin));
    partial_data3           = (calib_data->quantized_calib_data.par_p8 * (calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin));
    partial_out1            = calib_data->quantized_calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1           = (calib_data->quantized_calib_data.par_p2 * calib_data->quantized_calib_data.t_lin);
    partial_data2           = (calib_data->quantized_calib_data.par_p3 * (calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin));
    partial_data3           = (calib_data->quantized_calib_data.par_p4 * (calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin * calib_data->quantized_calib_data.t_lin));
    partial_out2            = (float) uncomp_press * (calib_data->quantized_calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1           = ((float) uncomp_press * (float) uncomp_press);
    partial_data2           = (calib_data->quantized_calib_data.par_p9 + calib_data->quantized_calib_data.par_p10 * calib_data->quantized_calib_data.t_lin);
    partial_data3           = partial_data1 * partial_data2;
    partial_data4           = partial_data3 + ((float) uncomp_press * (float) uncomp_press * (float) uncomp_press) * calib_data->quantized_calib_data.par_p11;

    comp_press              = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
} // end of BMP390_compensate_pressure ---------------------
#endif


void pause_ms(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}


#if (USES_AHT > 0)
uint16_t  i2c_AHT_write(uint8_t addr, uint8_t* buf, uint16_t len)
{
    uint16_t sent       = 0;

    i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return sent;
}


uint16_t    i2c_AHT_read(uint8_t addr, uint8_t from, uint8_t* buf, uint16_t len)
{
    uint16_t    read        = len;
    uint8_t     send_buf[2];

    send_buf[0]             = from;

    i2c_master_write_to_device(I2C_MASTER_NUM, addr, send_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, addr, buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return read;
}
#endif

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port     = I2C_MASTER_NUM;

    i2c_config_t conf       = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = I2C_MASTER_SDA_IO,
        .scl_io_num         = I2C_MASTER_SCL_IO,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    // return i2c_master_bus_add_device();
}


#if (USE_MQTT > 0)

static void     publish_2_mqtt(esp_mqtt_client_handle_t client, char* topic, char* payload)
{
    int len                 = strlen(payload);
    int msg_id              = esp_mqtt_client_publish(client, topic, payload, len, 0, 0);
    ESP_LOGI(MQTT_TAG, "published with msg_id=%d", msg_id);
}


/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event       = event_data;
    // esp_mqtt_client_handle_t client     = event->client;
    mqtt_handle                         = event->client;
//    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // msg_id                          = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id                          = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id                          = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        mqtt.enabled				= 1;

        fv_sensor_write_config(1);
        // fv_sensor_write_config(2);
        // fv_sensor_write_config(3);
        // fv_sensor_write_config(4);
        publish_2_mqtt(mqtt_handle, "/HVAC/intake/config", json_payload);       // TODO: update with auto discovery topic
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt.enabled				= 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id                          = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


/***********************************************************
	@brief	sets up the MQTT client and registers the event handler.
***********************************************************/
static void		mqtt_app_start(void)
{
	if (mqtt_handle != NULL)
    {
		esp_mqtt_client_stop(mqtt_handle);
	}
    
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = CONFIG_BROKER_URI,
        },
        .credentials = {
            // .username   = "mqtt-user",
            .username   = mqtt.username,
            // .authentication.password = "DB17ms1pw"
            .authentication.password = mqtt.password
        },
    };

    ESP_LOGI(MQTT_TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    mqtt_handle                         = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_handle);

    printf("MQTT started\n");
    int outbox_size			            = esp_mqtt_client_get_outbox_size(mqtt_handle);
    printf("MQTT outbox size %u bytes\n", outbox_size);
} // end of mqtt_app_start ---------------------------------
#endif	// (USE_MQTT > 0)


#if (USE_MQTT > 0)
void            fv_sensor_write_json(float temp, float delta, float absolute)
{
    bool blocked            = false;
    if (delta > mqtt.filter_threshold)
    {
        blocked             = true;
    }
    sprintf(json_payload, JSON_SENSOR_READING, temp, absolute / 100.0, delta, blocked);
} // end of fv_sensor_write_json ---------------------------
#endif	// (USE_MQTT > 0)


#if (USE_MQTT > 0)
/*
{
  "name": "intake_temp",
  "unique_id": "intake_3FFBA93D_1",
  "state_topic": "temperature",
  "unit_of_measurement": "°C",
  "device": {
    "name": "HVACintake",
    "ids": "intake_3FFBA93D",
    "sw": "v0.1"
  }
}

{
  "name": "intake_press",
  "unique_id": "intake_3FFBA93D_2",
  "state_topic": "pressure",
  "unit_of_measurement": "hPa",
  "device": {
    "name": "HVACintake",
    "ids": "intake_3FFBA93D"
  }
}
*/
void            fv_sensor_write_config(uint8_t element_num)
{
    char    ca_device[80];
    char    ca_item_device[65];
    char    element_id[23];
    char    sensor_name[15];
    char    element_name[18];

    sprintf(sensor_name, "HVAC%s", SENSOR_TEXT);

    // the element id is the device id + "_1", "_2" etc
    sprintf(element_id, mqtt.devicename);
    strcat(element_id, "_");
    uint8_t index               = strlen(element_id);
    element_id[index]           = ('0' + element_num);
    element_id[index + 1]       = '\0';
    
    sprintf(ca_device, JSON_DISCOVERY_DEVICE_CORE, sensor_name, mqtt.devicename, "v0.1");
    sprintf(ca_item_device, JSON_DISCOVERY_DEVICE_ITEM, sensor_name, mqtt.devicename);

    switch (element_num)
    {
        case    1:
#if ( (HVAC_SENSOR_INTAKE > 0) || (HVAC_SENSOR_EXTRACT > 0) )
        sprintf(element_name, "%s_temp", SENSOR_TEXT);
#endif
        sprintf(json_payload, JSON_DISCOVERY_CONFIG, element_name, element_id, "temperature", "°C", &ca_device[0]);
        break;

        case    2:
#if ( (HVAC_SENSOR_INTAKE > 0) || (HVAC_SENSOR_EXTRACT > 0) )
        sprintf(element_name, "%s_press", SENSOR_TEXT);
#endif
        sprintf(json_payload, JSON_DISCOVERY_CONFIG, element_name, element_id, "pressure", "hPa", &ca_item_device[0]);
        break;
        
        case    3:
#if ( (HVAC_SENSOR_INTAKE > 0) || (HVAC_SENSOR_EXTRACT > 0) )
        sprintf(element_name, "%s_filter", SENSOR_TEXT);
#endif
        sprintf(json_payload, JSON_DISCOVERY_CONFIG, element_name, element_id, "pressure", "Pa", &ca_item_device[0]);
        break;
        
        case    4:
#if ( (HVAC_SENSOR_INTAKE > 0) || (HVAC_SENSOR_EXTRACT > 0) )
        sprintf(element_name, "%s_blocked", SENSOR_TEXT);
#endif
        sprintf(json_payload, JSON_DISCOVERY_CONFIG, element_name, element_id, "bool", "", &ca_item_device[0]);
        break;

        case    5:
#if (HVAC_SENSOR_EXTRACT > 0)
        sprintf(element_name, "%s_humidity", SENSOR_TEXT);
#endif
        break;

        case    6:
#if (HVAC_SENSOR_EXTRACT > 0)
        sprintf(element_name, "%s_co2", SENSOR_TEXT);
#endif
        break;

        default:
        break;
    }
} // end of fv_sensor_write_json ---------------------------
#endif	// (USE_MQTT > 0)


#if (USE_MQTT > 0)
/***********************************************************
	Builds a webpage form to configure the mqtt client.
***********************************************************/
void				fv_mqtt_config_form(char *webpage)
{
	// Head of the HTML page
	sprintf(webpage, "<!DOCTYPE html><html><body><h1>MQTT Setup</h1>");		// 46
	strcat(webpage, "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");	// 71
	strcat(webpage, "<p style=\"font-size:20px\"><font size=\"6\">");		// 41
	strcat(webpage, "<form>");												// 6, 164

	// populate a table
	strcat(webpage, "<table>");												// 7
	strcat(webpage, "<tr><th>Name</th><th>Value</th></tr>");				// 36
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Hostname</td>");	// 47
	strcat(webpage, "<td><input name=\"hostname\" value=\"");				// 34
	char TableRow[80];
	sprintf(TableRow, "%s\"></td></tr>", mqtt.hostname);					// 12+30 = 42, 330
	strcat(webpage, TableRow);
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Port</td>");		// 43
	strcat(webpage, "<td><input type=\"number\" name=\"port\" min=\"0\" max=\"65535\" value=\"");	// 64
	sprintf(TableRow, "%d\"></td></tr>", mqtt.Port);						// 12+5 = 17, 454
	strcat(webpage, TableRow);
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Username</td>");	// 47
	strcat(webpage, "<td><input name=\"username\" value=\"");				// 34
	sprintf(TableRow, "%s\"></td></tr>", mqtt.username);					// 12+50 = 62, 597
	strcat(webpage, TableRow);
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Password</td>");	// 47
	strcat(webpage, "<td><input type=\"password\" name=\"password\" value=\"");		// 50
	sprintf(TableRow, "%s\"></td></tr>", mqtt.password);					// 12+30 = 42, 700
	strcat(webpage, TableRow);
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Site name</td>");	// 48
	strcat(webpage, "<td><input name=\"sitename\" value=\"");				// 34
	sprintf(TableRow, "%s\"></td></tr>", mqtt.devicename);					// 12+30 = 42, 
	strcat(webpage, TableRow);
	
	strcat(webpage, "<tr style=\"text-align:center;\"><td>Site ID</td>");	// 46
	strcat(webpage, "<td><input type=\"number\" name=\"siteid\" min=\"0\" max=\"4294967295\" value=\"");	// 71
	sprintf(TableRow, "%ld\"></td></tr>", mqtt.id);						// 12+10 = 22, 
	strcat(webpage, TableRow);
	
	// strcat(webpage, "<tr style=\"text-align:center;\"><td>Min Interval</td>");		// 55
	// strcat(webpage, "<td><input type=\"number\" name=\"mininterval\" min=\"10\" max=\"255\" value=\"");	// 74
	// sprintf(TableRow, "%d\"></td></tr>", mqtt.Min_Interval);						// 12+3 = 15, 
	// strcat(webpage, TableRow);
	
	strcat(webpage, "<tr><td colspan=\"2\"><hr></td></tr>");				// 34,
	
	strcat(webpage, "<tr><td colspan=\"2\"><hr></td></tr>");				// 34, 1097

	strcat(webpage, "<tr><td colspan=\"2\"><input type=\"radio\" name=\"enabled\" value=\"0\"");	// 42, 
	if (mqtt.enabled == 0) {
		strcat(webpage, " checked");										// 8, 
	}
	strcat(webpage, "> MQTT disabled</td></tr>");							// 25, 1172

	strcat(webpage, "<tr><td colspan=\"2\"><input type=\"radio\" name=\"enabled\" value=\"1\"");	// 42, 
	if (mqtt.enabled == 1) {
		strcat(webpage, " checked");										// 8, 
	}
	strcat(webpage, "> MQTT enabled</td></tr>");							// 24, 1246
	
	strcat(webpage, "<tr><td colspan=\"2\"><hr></td></tr>");				// 34, 1280

	strcat(webpage, "<tr><td colspan=\"2\"><b>Assign the settings in the table above</b></td></tr>");	// 75
	strcat(webpage, "<tr><td colspan=\"2\"><b>and press update to apply them.</b></td></tr>");	// 69, 1424

	// add the submit button
	strcat(webpage, "</table><br/><input type=\"submit\" value=\"Update\" style=\"height:50px; width:100px\" />");	// 83
	// add the reset button
	strcat(webpage, "<input type=\"reset\" style=\"height:50px; width:100px\" />");	// 55, 1562

	// footer of the webpage
	strcat(webpage, "</form></body></html>");								// 21, 1583
} // end of fv_mqtt_config_form ----------------------------
#endif


/***********************************************************
	Builds a webpage form to configure the WiFi.
***********************************************************/
void			fv_wifi_config_form(char *webpage)
{
	// Head of the HTML page
	sprintf(webpage, "<!DOCTYPE html><html><body><h1>WiFi Configuration</h1>");		// 54
	strcat(webpage, "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");	// 71, 125
	strcat(webpage, "<p style=\"font-size:20px\"><font size=\"6\">");		// 41, 166
	strcat(webpage, "<form>");								// 6, 172

	if (num_APs == 0)
    {
		strcat(webpage, "<tr><th><b>No Access Points Found!</b></th></tr>");
	}
	else {
		// populate a table of APs.
		strcat(webpage, "<table>");							// 7, 179
		for (int i = 0; i < num_APs; ++i)
        {
			char TableRow[100];
			sprintf(TableRow, "<tr><td><input type=\"radio\" name=\"AP\" value=\"%d\"", i + 1);	// 47
			if (i == 0) {
				strcat(TableRow, " checked> ");				// 10, 57
			}
			else {
				strcat(TableRow, "> ");						// 2, 49
			}
			strcat(TableRow, AP_list[i].ssid);				// 32, 89, 81
			strcat(TableRow, "</td></tr>");					// 10, 99, 91

			strcat(webpage, TableRow);						// add the table row to the page
		} // end of for each AP
		// (91 * 8) + 99 = 827 bytes, 1006

		strcat(webpage, "<tr><td><b>Select your WiFi from the list above.</b></td></tr>");	// 62, 1068

		// TODO: could add a textbox here for hidden SSID entry. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		strcat(webpage, "<tr><td>Or enter your hidden WiFi in the box below.</td></tr>");	// 61, 1129
		strcat(webpage, "<tr><td>");						// 8, 1137
		strcat(webpage, "<input name=\"HIDDEN\"");			// 20, 1157
		strcat(webpage, " value=\"\"");						// 9, 1166
		strcat(webpage, " Style=\"background-color: #98FB98;\"");	// 35, 1201
		strcat(webpage, " /></td></tr>");					// 13, 1214

		strcat(webpage, "<tr><td><hr></td></tr>");			// 22, 1090
		strcat(webpage, "<tr><td><b>Input the password for your WiFi.</b></td></tr>");	// 58, 1148

		// build the HTML code for a table row with configuration value name and the value itself inside a textbox
		strcat(webpage, "<tr><td>");						// 8, 1156
		strcat(webpage, "<input name=\"PSK\"");				// 17, 1173
		strcat(webpage, " value=\"\"");						// 9, 1182
		strcat(webpage, " Style=\"background-color: #98FB98;\"");	// 35, 1217
		strcat(webpage, " /></td></tr>");					// 13, 1230

		// add the submit button
		strcat(webpage, "</table><br/><input type=\"submit\" value=\"Connect\" style=\"height:50px; width:100px\" />");	// 84, 1314
	} // end of else APs available

	// footer of the webpage
	strcat(webpage, "</form></body></html>");				// 21, 1335
} // end of fv_wifi_config_form ----------------------------


#if (USES_MHZ > 0)
uint32_t millis(void)
{
    TickType_t  tick_count      = 0;

    tick_count                  = xTaskGetTickCount();

    return (tick_count * portTICK_PERIOD_MS);
} // end of millis -----------------------------------------


void serial_MHZ_flush(void)
{
    uart_flush_input(EX_UART_NUM);
} // end of serial_MHZ_flush -------------------------------


uint16_t serial_MHZ_available(void)
{
    size_t    bytes_avail     = 0;

    uart_get_buffered_data_len(EX_UART_NUM, &bytes_avail);

    return (uint16_t) bytes_avail;
} // end of serial_MHZ_available ---------------------------


uint16_t serial_MHZ_read(uint8_t* buf)
{
    uint16_t    bytes_read      = 0;
    uint16_t    buf_len         = sizeof(buf);

    bytes_read                  = uart_read_bytes(EX_UART_NUM, buf, buf_len, 100 / portTICK_PERIOD_MS);

    return bytes_read;
} // end of serial_MHZ_read --------------------------------


uint16_t serial_MHZ_write(uint8_t* buf, uint16_t len)
{
    uint16_t    bytes_written   = 0;

    bytes_written               = uart_write_bytes(EX_UART_NUM, (const char*) buf, len);

    return bytes_written;
} // end of serial_MHZ_write -------------------------------
#endif


#if (USE_MDNS != 0)
/***********************************************************
	@brief	sets up the mDNS service.

	checks if the default hostname is already assigned,
		if so, it appends a number.
***********************************************************/
void			mdns_setup()
{
	// initialize mDNS service
	esp_err_t err					= mdns_init();
	if (err)
    {
		printf("mDNS Init failed: %d\n", err);
		return;
	}

	// test if the hostname is already in use on the network.
	if (ESP_OK == resolve_mdns_host(hostname))
    {
		// append a number and try again.
		int suff_idx				= 0;
		while ( (suff_idx < (HOSTNAME_LEN - 3))				// ensure enough room for "-2"
				&& (hostname[suff_idx] >= ' ')				// scroll past valid chars
				&& (hostname[suff_idx] != '-') )			// stop if you find a hyphen already.
        {
			++suff_idx;
		}
		hostname[suff_idx]			= '-';
		++suff_idx;
		hostname[suff_idx]			= '2';
		hostname[suff_idx + 1]		= 0;					// string terminator

		uint8_t suffix				= 2;
		while ( (ESP_OK == resolve_mdns_host(hostname))
				&& (suffix < 9) )
        {
			++suffix;
			hostname[suff_idx]		= (suffix + '0');
		} // end of while hostname exists
		
		// update the stored hostname because it changed.
//		fv_sd_card_write_config();
	} // end of if hostname already exists
	
	mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
#if (USE_HTTPS > 0)
	mdns_service_add(NULL, "_http", "_tcp", 443, NULL, 0);
#endif

	printf("mDNS hostname is %s\n", hostname);

    // set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(hostname) );
	// set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set(hostname) );
	mdns_service_instance_name_set("_http", "_tcp", hostname);
} // end of mdns_setup -------------------------------------


/***********************************************************
	@brief	checks if the hostname exists on the network.
	@param	host_name:	the hostname to check for.
	@retval	returns ESP_OK if the hostname exists.
***********************************************************/
esp_err_t		resolve_mdns_host(const char * host_name)
{
    esp_ip4_addr_t		addr;
    addr.addr			= 0;

    esp_err_t err		= mdns_query_a(host_name, 2000, &addr);
    if (err) {
        if (err == ESP_ERR_NOT_FOUND)
        {
        }
		else
        {
			printf("mdns Query Failed\n");
        }
    }
	else
    {
		printf(IPSTR, IP2STR(&addr));
		printf(" = %s.local\n", host_name);
	}

	return err;
} // end of resolve_mdns_host ------------------------------
#endif  // (USE_MDNS != 0)


/***********************************************************
	@brief	Scans the WiFi for Access Points (APs)
***********************************************************/
void			fv_main_scan_for_APs(void)
{
	printf("Scanning for APs.\n");
	u8_scan_for_APs_f					= 1;
	esp_wifi_disconnect();

	esp_wifi_stop();
	strSSID[0]							= 0;
	strPassword[0]						= 0;
//	wifi_setup(1);      // TODO: need to set up wifi for scanning

	if (esp_wifi_scan_start(&scan_config, 1) != ESP_OK)
    {
		printf("Scan error!\n");
	}
	esp_wifi_scan_get_ap_num(&num_APs);
	printf("%d APs found.\n", num_APs);

	if (num_APs != 0)
    {
		wifi_ap_record_t	AP_records[num_APs];			// struct for the AP information
		esp_wifi_scan_get_ap_records(&num_APs, AP_records);
		if (num_APs > MAX_AP_LIST_ITEMS)
        {
			num_APs						= MAX_AP_LIST_ITEMS;
		}
		for (int i = 0; i < num_APs; ++i)
        {

			printf("%d ", i + 1);

			for (int j = 0; j < SSID_LEN; ++j)
            {
				AP_list[i].ssid[j]		= AP_records[i].ssid[j];

				printf("%c", AP_list[i].ssid[j]);
			}
			AP_list[i].rssi				= AP_records[i].rssi;

			printf(", %d\n", AP_list[i].rssi);
		} // end of for each AP found
	} // end of if APs found
	u8_scan_for_APs_f					= 0;
} // end of fv_main_scan_for_APs ---------------------------


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        disp_content.connected                      = 0;

        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        disp_content.connected                      = 1;

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta(void)
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
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


/*
    MM   MM    A    III NN  N
    M M M M   A A    I  N N N
    M  M  M  AAAAA   I  N  NN
    M     M A     A III N   N
*/
void app_main(void)
{
    uint8_t data[10];
    uint8_t sensor_id                           = 0;
    uint8_t sensor_num                          = (sensor_id + 1);
    uint16_t    second_ctr                      = 0;
    float   sensor_temp                         = 0.0;
    float   sensor_filter_delta                 = 0.0;
    float   sensor_pressure1                    = 0.0;
    float   sensor_pressure2                    = 0.0;

#if (USES_LCD_SMS1706 > 0)
    disp_content.battery                        = 0;
    disp_content.connected                      = 0;
    disp_content.exclamation                    = 0;
    disp_content.line                           = DISPLAY_LINE_0;
    disp_content.lock                           = DISPLAY_LOCKS_0;
    disp_content.p_arrow                        = DISPLAY_ARROW_NONE;
    disp_content.q_arrow                        = DISPLAY_ARROW_NONE;
    disp_content.sig_strength                   = DISPLAY_SIGNAL_0;
    // disp_content.units                          = DISPLAY_UNIT_A;
    // disp_content.large_mode                     = DISPLAY_LARGE_NUMBER;
    disp_content.units                          = DISPLAY_UNIT_OFF;
    disp_content.large_mode                     = DISPLAY_LARGE_TEXT;
    disp_content.large_num_max_dp               = 0;
    disp_content.t_reg_num                      = 0;
    disp_content.g_reg_num                      = 0;
    disp_content.pr_reg_num                     = 0;
    disp_content.relay                          = DISPLAY_LINE_0;
#endif

    //Initialize NVS
    esp_err_t ret            = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret                       = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI("HVAC sensors", "IDF version: %s", esp_get_idf_version());

#if (USE_MQTT > 0)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
	// Set the defaults
	mqtt.enabled				= 0;
	mqtt.Port					= 1883;
	sprintf(mqtt.hostname, "homeassistant.local");
	sprintf(mqtt.username, "mqtt-user");
	sprintf(mqtt.password, "DB17ms1pw");
    sprintf(mqtt.devicename, "%s_%06X", SENSOR_TEXT, (unsigned int) &mac[3]);
    sprintf(mqtt.sensor_topic, MQTT_SENSOR_TOPIC, SENSOR_TEXT);
	mqtt.id					    = 123456789;
    mqtt.filter_threshold       = 150;
    mqtt.filter_offset          = 8.0;
    mqtt.interval               = 300;          // every 5 minutes
#endif

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

#if (USES_LCD_SMS1706 > 0)
    fv_display_on();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    fv_display_set_V(100);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    fv_display_blink(DISPLAY_BLINK_OFF);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    fv_display_all_on();
#endif

#if ( (HVAC_SENSOR_INTAKE > 0) || (HVAC_SENSOR_EXTRACT > 0) )
    // test for multiple I2C sensors
    for (sensor_id = 0; sensor_id < 2; ++sensor_id)
    {
#if (USES_BMP3 > 0)
        bmp390_dev[sensor_id].intf              = BMP3_I2C_INTF;
        bmp390_dev[sensor_id].read              = sensor_register_read;
        bmp390_dev[sensor_id].delay_us          = vTaskDelay;
#endif

        /* Read the sensor ID register */
#if (USES_BMP3 > 0)
        sensor_register_read(sensor_id + 1, BMP3_REG_CHIP_ID, data, 1);
#endif
#if (USES_BME2 > 0)
        sensor_register_read(sensor_id + 1, BME280_REG_CHIP_ID, data, 1);
#endif
        ESP_LOGI(TAG, "sensor ID = 0x%02X", data[0]);

#if (USES_BMP3 > 0)
        if (data[0] == BMP390_CHIP_ID)
        {
            bmp390_dev[sensor_id].chip_id       = data[0];

            uint8_t sensor_num                  = (sensor_id + 1);

            printf("BMP390 detected on sensor %u\n", sensor_num);
            
            get_calib_data(sensor_num, &bmp390_dev[sensor_id]);

            // sensor_register_write_byte(sensor_num,
            //                             BMP3_REG_OSR,
            //                             (BMP3_OVERSAMPLING_4X));    // oversample the pressure readings only

            sensor_register_write_byte(sensor_num,
                                        BMP3_REG_CONFIG,
                                        (BMP3_IIR_FILTER_COEFF_63 << BMP3_IIR_FILTER_POS));

            sensor_register_write_byte(sensor_num,
                                        BMP3_REG_PWR_CTRL,
                                        (BMP390_CNTRL_MODE_NORMAL | BMP390_CNTRL_PRESS_EN | BMP390_CNTRL_TEMP_EN));

            sensor_register_read(sensor_num, BMP3_REG_PWR_CTRL, data, 1);
            printf("BMP390 PWR_CNTRL register 0x%02X\n", data[0]);
        }
#endif

#if (USES_BME2 > 0)
        if (data[0] == BME280_CHIP_ID)
        {
            uint8_t sensor_num                  = (sensor_id + 1);
            printf("BME280 detected on sensor %u\n", sensor_num);
        }
#endif
    }
#endif

#if (USES_MHZ > 0)
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate              = MHZ_BAUDRATE,
        .data_bits              = UART_DATA_8_BITS,
        .parity                 = UART_PARITY_DISABLE,
        .stop_bits              = UART_STOP_BITS_1,
        .flow_ctrl              = UART_HW_FLOWCTRL_DISABLE,
        .source_clk             = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    MHZ_init(MHZ14A, RANGE_5K);
    MHZ_setDebug(1);
#endif

#if (USES_AHT > 0)
    if (AHTxx(AHT10_ADDRESS_X39, AHT1x_SENSOR) != 0)
    {
        printf("AHT sensor NOT initialised!");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
#endif
    
    sensor_id                           = 0;

#if (USE_MQTT != 0)
	mqtt_app_start();
#endif

    printf("[APP] Free memory: %lu bytes\n", esp_get_free_heap_size());

    while (1)
    {
        vTaskDelay(6000 / portTICK_PERIOD_MS);
        second_ctr                      += 6;
        
        sensor_num                      = (sensor_id + 1);

#if (USES_BMP3 > 0)
        if (bmp390_dev[sensor_id].chip_id == BMP390_CHIP_ID)
        {
            sensor_register_read(sensor_num, BMP3_REG_DATA, data, 6);
            int uncomp_press        = (data[2] << 16) + (data[1] << 8) + data[0];
            int uncomp_temp         = (data[5] << 16) + (data[4] << 8) + data[3];

            float temperature       = BMP390_compensate_temperature(uncomp_temp, &bmp390_dev[sensor_id].calib_data);
            // use the first sensor for temperature unless it's not available
            if ( (sensor_num == 1)
                    || (bmp390_dev[0].chip_id != BMP390_CHIP_ID) )
            {
                sensor_temp         = temperature;
            }
            printf("BMP390 %u temperature %3.1f C\n", sensor_num, temperature);
#if (USES_LCD_SMS1706 > 0)
            sprintf(disp_content.small_text, "%3.1fc", temperature);
#endif
            float pressure          = BMP390_compensate_pressure(uncomp_press, &bmp390_dev[sensor_id].calib_data);
            if (sensor_num == 1)
            {
                sensor_pressure1    = pressure;
            }
            else
            {
                sensor_pressure2    = pressure;
            }
            printf("BMP390 %u pressure %5.0f Pa\n", sensor_num, pressure);
#if (USES_LCD_SMS1706 > 0)
            disp_content.large_number   = pressure;
            sprintf(disp_content.large_text, "%3.0fhpA", pressure / 100.0);
#endif
        }
#endif

#if (USES_LCD_SMS1706 > 0)
        disp_content.t_reg_num          = sensor_num;

        if (disp_content.connected == 1)
        {
            int rssi;
            esp_wifi_sta_get_rssi(&rssi);
            printf("rssi: %d\n", rssi);
            if (rssi > -15)
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_5;    // 0-5
            }
            else if (rssi > -30)
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_4;    // 0-5
            }
            else if (rssi > -45)
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_3;    // 0-5
            }
            else if (rssi > -60)
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_2;    // 0-5
            }
            else if (rssi > -75)
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_1;    // 0-5
            }
            else
            {
                disp_content.sig_strength   = DISPLAY_SIGNAL_0;    // 0-5
            }
        }
        else
        {
            disp_content.sig_strength   = DISPLAY_SIGNAL_0;    // 0-5
        }
        fv_display_content(disp_content);
#endif

        ++sensor_id;
        if (sensor_id > 1)
        {
            sensor_id                   = 0;

#if (USES_MHZ > 0)
            readCO2UART();
#endif
        }

        if (second_ctr >= mqtt.interval)
        {
            second_ctr                  = 0;
            
#if (USE_MQTT > 0)
            if (mqtt.enabled == 0)
            {
                mqtt_app_start();
            }
            else
            {
                // ensure both sensors are reading Pascals
                if ( (sensor_pressure1 > 1000.0) && (sensor_pressure2 > 1000.0) )
                {
                    // update the filter pressure drop with an IIR filter
                    sensor_filter_delta     = (((sensor_pressure1 - sensor_pressure2) - mqtt.filter_offset) + sensor_filter_delta) / 2.0;
                }
                fv_sensor_write_json(sensor_temp, sensor_filter_delta, sensor_pressure1);
                publish_2_mqtt(mqtt_handle, mqtt.sensor_topic, json_payload);
            }
#endif
        }
    }
}
