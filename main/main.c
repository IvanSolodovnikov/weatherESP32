#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000

#define AHT20_ADDR 0x38
#define BMP280_ADDR 0x77
#define BH1750_ADDR_PRIMARY 0x23
#define BH1750_ADDR_SECONDARY 0x5C

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_PRESS_MSB 0xF7

#define BH1750_CMD_POWER_ON 0x01
#define BH1750_CMD_RESET 0x07
#define BH1750_CMD_CONT_HIGH_RES 0x10

#define SAMPLE_PERIOD_MS 5000
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAX_RETRY 10

static const char *TAG = "weather";

typedef struct {
    float aht_temperature;
    float humidity;
    float bmp_temperature;
    float pressure;
    float lux;
    uint32_t uptime_s;
} sensor_snapshot_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    int32_t t_fine;
    uint8_t addr;
} bmp280_t;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t aht20_dev;
static i2c_master_dev_handle_t bh1750_dev;
static uint8_t bh1750_addr;
static SemaphoreHandle_t weather_lock;
static sensor_snapshot_t current_weather = {
    .aht_temperature = NAN,
    .humidity = NAN,
    .bmp_temperature = NAN,
    .pressure = NAN,
    .lux = NAN,
};
static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count;

static esp_err_t i2c_add_device(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    return i2c_master_bus_add_device(i2c_bus, &dev_config, dev);
}

static esp_err_t i2c_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len)
{
    return i2c_master_transmit(dev, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t i2c_read(i2c_master_dev_handle_t dev, uint8_t *data, size_t len)
{
    return i2c_master_receive(dev, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    return i2c_write(dev, data, sizeof(data));
}

static uint16_t u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t s16_le(const uint8_t *data)
{
    return (int16_t)u16_le(data);
}

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&config, &i2c_bus);
}

static void weather_store_current(float aht_temperature, float humidity, float bmp_temperature, float pressure, float lux)
{
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    xSemaphoreTake(weather_lock, portMAX_DELAY);

    current_weather.aht_temperature = aht_temperature;
    current_weather.humidity = humidity;
    current_weather.bmp_temperature = bmp_temperature;
    current_weather.pressure = pressure;
    current_weather.lux = lux;
    current_weather.uptime_s = uptime_s;

    xSemaphoreGive(weather_lock);
}

static const char *json_float(char *buf, size_t len, float value)
{
    if (isnan(value)) {
        return "null";
    }

    snprintf(buf, len, "%.2f", value);
    return buf;
}

static void i2c_scan(void)
{
    bool found = false;

    ESP_LOGI(TAG, "Scanning I2C bus SDA=%d SCL=%d...", I2C_SDA_GPIO, I2C_SCL_GPIO);
    ESP_LOGI(TAG, "I2C idle levels before scan: SDA=%d SCL=%d",
             gpio_get_level(I2C_SDA_GPIO),
             gpio_get_level(I2C_SCL_GPIO));

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
            found = true;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "No I2C devices found. Check VCC, GND, SDA/SCL pins and pull-up resistors.");
    }

    ESP_LOGI(TAG, "Probe 0x%02X result: %s", AHT20_ADDR,
             esp_err_to_name(i2c_master_probe(i2c_bus, AHT20_ADDR, 100)));
    ESP_LOGI(TAG, "Probe 0x%02X result: %s", BMP280_ADDR,
             esp_err_to_name(i2c_master_probe(i2c_bus, BMP280_ADDR, 100)));
    ESP_LOGI(TAG, "Probe 0x%02X result: %s", BH1750_ADDR_PRIMARY,
             esp_err_to_name(i2c_master_probe(i2c_bus, BH1750_ADDR_PRIMARY, 100)));
    ESP_LOGI(TAG, "Probe 0x%02X result: %s", BH1750_ADDR_SECONDARY,
             esp_err_to_name(i2c_master_probe(i2c_bus, BH1750_ADDR_SECONDARY, 100)));
    ESP_LOGI(TAG, "I2C idle levels after scan: SDA=%d SCL=%d",
             gpio_get_level(I2C_SDA_GPIO),
             gpio_get_level(I2C_SCL_GPIO));
}

static esp_err_t aht20_init(void)
{
    uint8_t status = 0;
    vTaskDelay(pdMS_TO_TICKS(40));

    ESP_RETURN_ON_ERROR(i2c_master_probe(i2c_bus, AHT20_ADDR, 100), TAG, "AHT20 probe failed");
    ESP_RETURN_ON_ERROR(i2c_add_device(AHT20_ADDR, &aht20_dev), TAG, "AHT20 add device failed");
    ESP_RETURN_ON_ERROR(i2c_read(aht20_dev, &status, 1), TAG, "AHT20 status read failed");

    if ((status & 0x08) == 0) {
        const uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
        ESP_RETURN_ON_ERROR(i2c_write(aht20_dev, init_cmd, sizeof(init_cmd)), TAG, "AHT20 init failed");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t aht20_read(float *temperature_c, float *humidity_percent)
{
    const uint8_t measure_cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[7] = {0};

    ESP_RETURN_ON_ERROR(i2c_write(aht20_dev, measure_cmd, sizeof(measure_cmd)), TAG, "AHT20 measure start failed");
    vTaskDelay(pdMS_TO_TICKS(80));
    ESP_RETURN_ON_ERROR(i2c_read(aht20_dev, data, sizeof(data)), TAG, "AHT20 read failed");

    if (data[0] & 0x80) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temperature = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    *humidity_percent = ((float)raw_humidity * 100.0f) / 1048576.0f;
    *temperature_c = ((float)raw_temperature * 200.0f) / 1048576.0f - 50.0f;

    return ESP_OK;
}

static esp_err_t bh1750_init(void)
{
    bh1750_addr = BH1750_ADDR_PRIMARY;
    esp_err_t err = i2c_master_probe(i2c_bus, bh1750_addr, 100);

    if (err != ESP_OK) {
        bh1750_addr = BH1750_ADDR_SECONDARY;
        err = i2c_master_probe(i2c_bus, bh1750_addr, 100);
    }

    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(i2c_add_device(bh1750_addr, &bh1750_dev), TAG, "BH1750 add device failed");

    const uint8_t power_on = BH1750_CMD_POWER_ON;
    const uint8_t reset = BH1750_CMD_RESET;
    const uint8_t mode = BH1750_CMD_CONT_HIGH_RES;

    ESP_RETURN_ON_ERROR(i2c_write(bh1750_dev, &power_on, 1), TAG, "BH1750 power on failed");
    ESP_RETURN_ON_ERROR(i2c_write(bh1750_dev, &reset, 1), TAG, "BH1750 reset failed");
    ESP_RETURN_ON_ERROR(i2c_write(bh1750_dev, &mode, 1), TAG, "BH1750 mode set failed");
    vTaskDelay(pdMS_TO_TICKS(180));

    ESP_LOGI(TAG, "BH1750 found at 0x%02X", bh1750_addr);
    return ESP_OK;
}

static esp_err_t bh1750_read(float *lux)
{
    uint8_t data[2] = {0};

    ESP_RETURN_ON_ERROR(i2c_read(bh1750_dev, data, sizeof(data)), TAG, "BH1750 read failed");

    uint16_t raw_lux = ((uint16_t)data[0] << 8) | data[1];
    *lux = (float)raw_lux / 1.2f;

    return ESP_OK;
}

static esp_err_t bmp280_init(bmp280_t *bmp)
{
    uint8_t id = 0;
    uint8_t addr = BMP280_ADDR;
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_probe(i2c_bus, addr, 100);

    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(i2c_add_device(addr, &dev), TAG, "BMP280 add device failed");
    ESP_RETURN_ON_ERROR(i2c_read_reg(dev, BMP280_REG_ID, &id, 1), TAG, "BMP280 id read failed");

    if (id != 0x58) {
        ESP_LOGE(TAG, "Unexpected BMP280 chip id: 0x%02X", id);
        i2c_master_bus_rm_device(dev);
        return ESP_ERR_NOT_FOUND;
    }

    bmp->dev = dev;
    bmp->addr = addr;

    ESP_RETURN_ON_ERROR(i2c_write_reg(dev, BMP280_REG_RESET, 0xB6), TAG, "BMP280 reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t calib[24] = {0};
    ESP_RETURN_ON_ERROR(i2c_read_reg(dev, BMP280_REG_CALIB, calib, sizeof(calib)), TAG, "BMP280 calib read failed");

    bmp->dig_t1 = u16_le(&calib[0]);
    bmp->dig_t2 = s16_le(&calib[2]);
    bmp->dig_t3 = s16_le(&calib[4]);
    bmp->dig_p1 = u16_le(&calib[6]);
    bmp->dig_p2 = s16_le(&calib[8]);
    bmp->dig_p3 = s16_le(&calib[10]);
    bmp->dig_p4 = s16_le(&calib[12]);
    bmp->dig_p5 = s16_le(&calib[14]);
    bmp->dig_p6 = s16_le(&calib[16]);
    bmp->dig_p7 = s16_le(&calib[18]);
    bmp->dig_p8 = s16_le(&calib[20]);
    bmp->dig_p9 = s16_le(&calib[22]);

    ESP_RETURN_ON_ERROR(i2c_write_reg(dev, BMP280_REG_CONFIG, 0xA0), TAG, "BMP280 config failed");
    ESP_RETURN_ON_ERROR(i2c_write_reg(dev, BMP280_REG_CTRL_MEAS, 0x57), TAG, "BMP280 ctrl_meas failed");

    ESP_LOGI(TAG, "BMP280 found at 0x%02X", addr);
    return ESP_OK;
}

static float bmp280_compensate_temperature(bmp280_t *bmp, int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)bmp->dig_t1 << 1))) * (int32_t)bmp->dig_t2) >> 11;
    int32_t var2 = (((((adc_t >> 4) - (int32_t)bmp->dig_t1) * ((adc_t >> 4) - (int32_t)bmp->dig_t1)) >> 12) *
                    (int32_t)bmp->dig_t3) >>
                   14;

    bmp->t_fine = var1 + var2;
    return (float)((bmp->t_fine * 5 + 128) >> 8) / 100.0f;
}

static float bmp280_compensate_pressure(bmp280_t *bmp, int32_t adc_p)
{
    int64_t var1 = (int64_t)bmp->t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp->dig_p6;

    var2 += (var1 * (int64_t)bmp->dig_p5) << 17;
    var2 += (int64_t)bmp->dig_p4 << 35;
    var1 = ((var1 * var1 * (int64_t)bmp->dig_p3) >> 8) + ((var1 * (int64_t)bmp->dig_p2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * (int64_t)bmp->dig_p1 >> 33;

    if (var1 == 0) {
        return NAN;
    }

    int64_t pressure = 1048576 - adc_p;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)bmp->dig_p9 * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = ((int64_t)bmp->dig_p8 * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + ((int64_t)bmp->dig_p7 << 4);

    return (float)pressure / 256.0f;
}

static esp_err_t bmp280_read(bmp280_t *bmp, float *temperature_c, float *pressure_hpa)
{
    uint8_t status = 0;
    uint8_t data[6] = {0};

    do {
        ESP_RETURN_ON_ERROR(i2c_read_reg(bmp->dev, BMP280_REG_STATUS, &status, 1), TAG, "BMP280 status read failed");
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (status & 0x08);

    ESP_RETURN_ON_ERROR(i2c_read_reg(bmp->dev, BMP280_REG_PRESS_MSB, data, sizeof(data)), TAG, "BMP280 data read failed");

    int32_t adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    *temperature_c = bmp280_compensate_temperature(bmp, adc_t);
    *pressure_hpa = bmp280_compensate_pressure(bmp, adc_p) / 100.0f;

    return ESP_OK;
}

static const char index_html[] =
    "<!doctype html><html lang=\"ru\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Домашняя метеостанция</title>"
    "<style>"
    ":root{font-family:Arial,sans-serif;color:#182026;background:#eef3f5}"
    "body{margin:0}.wrap{max-width:1100px;margin:0 auto;padding:20px}"
    "h1{font-size:28px;margin:8px 0 18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px}"
    ".card{background:white;border:1px solid #d8e0e5;border-radius:8px;padding:14px}.label{font-size:13px;color:#66737c}"
    ".value{font-size:30px;font-weight:700;margin-top:8px}.unit{font-size:16px;color:#66737c;margin-left:3px}"
    ".status{color:#66737c;margin:0 0 14px}"
    "@media(max-width:640px){.wrap{padding:12px}.value{font-size:24px}}"
    "</style></head><body><main class=\"wrap\">"
    "<h1>Домашняя метеостанция</h1><p class=\"status\" id=\"status\">Загрузка...</p>"
    "<section class=\"grid\">"
    "<div class=\"card\"><div class=\"label\">AHT20 температура</div><div class=\"value\"><span id=\"aht_t\">--</span><span class=\"unit\">C</span></div></div>"
    "<div class=\"card\"><div class=\"label\">Влажность</div><div class=\"value\"><span id=\"hum\">--</span><span class=\"unit\">%</span></div></div>"
    "<div class=\"card\"><div class=\"label\">BMP280 температура</div><div class=\"value\"><span id=\"bmp_t\">--</span><span class=\"unit\">C</span></div></div>"
    "<div class=\"card\"><div class=\"label\">Давление</div><div class=\"value\"><span id=\"press\">--</span><span class=\"unit\">hPa</span></div></div>"
    "<div class=\"card\"><div class=\"label\">Освещенность</div><div class=\"value\"><span id=\"lux\">--</span><span class=\"unit\">lx</span></div></div>"
    "</section>"
    "</main><script>"
    "const fmt=v=>v===null?'--':Number(v).toFixed(2);"
    "async function load(){try{"
    "const c=await (await fetch('/api/current')).json();"
    "aht_t.textContent=fmt(c.aht_temperature);hum.textContent=fmt(c.humidity);bmp_t.textContent=fmt(c.bmp_temperature);press.textContent=fmt(c.pressure);lux.textContent=fmt(c.lux);"
    "status.textContent='Обновлено, uptime '+Math.floor(c.uptime_s/60)+' мин';"
    "}catch(e){status.textContent='Нет связи с ESP32';}}"
    "load();setInterval(load,5000);"
    "</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t current_handler(httpd_req_t *req)
{
    sensor_snapshot_t snapshot;
    char aht[16], hum[16], bmp[16], pressure[16], lux[16];
    char json[384];

    xSemaphoreTake(weather_lock, portMAX_DELAY);
    snapshot = current_weather;
    xSemaphoreGive(weather_lock);

    snprintf(json, sizeof(json),
             "{\"aht_temperature\":%s,\"humidity\":%s,\"bmp_temperature\":%s,\"pressure\":%s,\"lux\":%s,\"uptime_s\":%lu}",
             json_float(aht, sizeof(aht), snapshot.aht_temperature),
             json_float(hum, sizeof(hum), snapshot.humidity),
             json_float(bmp, sizeof(bmp), snapshot.bmp_temperature),
             json_float(pressure, sizeof(pressure), snapshot.pressure),
             json_float(lux, sizeof(lux), snapshot.lux),
             (unsigned long)snapshot.uptime_s);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    const httpd_uri_t current_uri = {
        .uri = "/api/current",
        .method = HTTP_GET,
        .handler = current_handler,
    };

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &current_uri);
    ESP_LOGI(TAG, "HTTP server started");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            wifi_retry_count++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "WiFi reconnect attempt %d", wifi_retry_count);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        wifi_retry_count = 0;
        ESP_LOGI(TAG, "WiFi connected, open http://" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void)
{
    if (strlen(CONFIG_WEATHER_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "WiFi SSID is empty. Run idf.py menuconfig -> Weather station");
        return ESP_ERR_INVALID_STATE;
    }

    wifi_event_group = xEventGroupCreate();
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG, "wifi event register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG, "ip event register failed");

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_WEATHER_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_WEATHER_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "wifi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(20000));

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "WiFi connection was not established yet");
    return ESP_ERR_TIMEOUT;
}

void app_main(void)
{
    bmp280_t bmp = {0};
    bool aht20_ready = false;
    bool bmp280_ready = false;
    bool bh1750_ready = false;
    esp_err_t nvs_err = nvs_flash_init();

    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    weather_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(weather_lock == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(i2c_init());
    i2c_scan();

    aht20_ready = (aht20_init() == ESP_OK);
    bmp280_ready = (bmp280_init(&bmp) == ESP_OK);
    bh1750_ready = (bh1750_init() == ESP_OK);

    if (!aht20_ready) {
        ESP_LOGW(TAG, "AHT20 not found at 0x%02X", AHT20_ADDR);
    }
    if (!bmp280_ready) {
        ESP_LOGW(TAG, "BMP280 not found at 0x%02X", BMP280_ADDR);
    }
    if (!bh1750_ready) {
        ESP_LOGW(TAG, "BH1750 not found at 0x%02X or 0x%02X", BH1750_ADDR_PRIMARY, BH1750_ADDR_SECONDARY);
    }

    if (wifi_init_sta() == ESP_OK) {
        start_webserver();
    } else {
        ESP_LOGW(TAG, "Web page is disabled until WiFi connects");
    }

    while (true) {
        float aht_temperature = NAN;
        float humidity = NAN;
        float bmp_temperature = NAN;
        float pressure = NAN;
        float lux = NAN;

        if (aht20_ready && aht20_read(&aht_temperature, &humidity) != ESP_OK) {
            ESP_LOGW(TAG, "AHT20 read failed");
        }

        if (bmp280_ready && bmp280_read(&bmp, &bmp_temperature, &pressure) != ESP_OK) {
            ESP_LOGW(TAG, "BMP280 read failed");
        }

        if (bh1750_ready && bh1750_read(&lux) != ESP_OK) {
            ESP_LOGW(TAG, "BH1750 read failed");
        }

        ESP_LOGI(TAG,
                 "AHT20: %.2f C, %.2f %%RH | BMP280: %.2f C, %.2f hPa | BH1750: %.2f lx",
                 aht_temperature,
                 humidity,
                 bmp_temperature,
                 pressure,
                 lux);

        weather_store_current(aht_temperature, humidity, bmp_temperature, pressure, lux);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
