#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"
#include "ui/level_indicator.h"
#include "esp_lvgl_port.h"

// I2C configuration
#define I2C_MASTER_SCL 10
#define I2C_MASTER_SDA 11
#define I2C_MASTER_NUM I2C_NUM_0
#define QMI8658_ADDRESS 0x6B // Replace with your QMI8658 address

SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;

static const char *TAG = "LevelTool"; // Define a tag for logging

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // I2C clock frequency
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void read_sensor_data(void* arg); // Function declaration

void setup_sensor() {
    i2c_master_init();

    // Initialize QMI8658 sensor with 4 parameters (port number, address, SDA, SCL)
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "Failed to find QMI8658 - check your wiring!");
        vTaskDelete(NULL); // Handle error gracefully
    }

    // Get chip ID
    ESP_LOGI(TAG, "Device ID: %x", qmi.getChipID());

    // Configure accelerometer
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true
    );

    // Configure gyroscope
    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    // Enable gyroscope and accelerometer
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(TAG, "Ready to read data...");
}

extern "C" void app_main() {
    // 初始化 LCD 和 触摸屏
    ui_main();
    // setup_sensor();
    // xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}

void read_sensor_data(void* arg) {
    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                // 将加速度数据转换为水平仪显示参数（范围-1.0~1.0）
                // 注意：根据传感器安装方向调整坐标轴转换
                float display_x = acc.x; // X轴对应1G量程
                float display_y = acc.y; // Y轴对应1G量程
                
                // 更新水平仪UI（需要LVGL线程锁）
                lvgl_port_lock(-1);
                update_level_indicator(display_x, display_y);
                lvgl_port_unlock();
                
                ESP_LOGI(TAG, "ACCEL: %f, %f, %f", acc.x, acc.y, acc.z);
            } else {
                ESP_LOGE(TAG, "Failed to read accelerometer data");
            }

            if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
                ESP_LOGI(TAG, "GYRO: %f, %f, %f", gyr.x, gyr.y, gyr.z);
            } else {
                ESP_LOGE(TAG, "Failed to read gyroscope data");
            }
            // Casting to unsigned int
            ESP_LOGI(TAG, "Timestamp: %u, Temperature: %.2f *C", (unsigned int)qmi.getTimestamp(), qmi.getTemperature_C());
        } else {
            ESP_LOGW(TAG, "Data not ready yet");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
