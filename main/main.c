#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000

#define AHT20_ADDR 0x38
#define BMP280_ADDR 0x77

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_PRESS_MSB 0xF7

static const char *TAG = "weather";

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

void app_main(void)
{
    bmp280_t bmp = {0};
    bool aht20_ready = false;
    bool bmp280_ready = false;

    ESP_ERROR_CHECK(i2c_init());
    i2c_scan();

    aht20_ready = (aht20_init() == ESP_OK);
    bmp280_ready = (bmp280_init(&bmp) == ESP_OK);

    if (!aht20_ready) {
        ESP_LOGW(TAG, "AHT20 not found at 0x%02X", AHT20_ADDR);
    }
    if (!bmp280_ready) {
        ESP_LOGW(TAG, "BMP280 not found at 0x%02X", BMP280_ADDR);
    }

    while (true) {
        float aht_temperature = NAN;
        float humidity = NAN;
        float bmp_temperature = NAN;
        float pressure = NAN;

        if (aht20_ready && aht20_read(&aht_temperature, &humidity) != ESP_OK) {
            ESP_LOGW(TAG, "AHT20 read failed");
        }

        if (bmp280_ready && bmp280_read(&bmp, &bmp_temperature, &pressure) != ESP_OK) {
            ESP_LOGW(TAG, "BMP280 read failed");
        }

        ESP_LOGI(TAG,
                 "AHT20: %.2f C, %.2f %%RH | BMP280: %.2f C, %.2f hPa",
                 aht_temperature,
                 humidity,
                 bmp_temperature,
                 pressure);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
