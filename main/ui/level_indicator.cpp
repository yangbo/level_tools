#include "ui/level_indicator.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl/lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lvgl_port.h"

// 显示硬件配置（根据实际硬件修改）
#define LCD_PIXEL_CLOCK_HZ   (10 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT 38

static lv_disp_drv_t disp_drv = {0};
static lv_indev_drv_t indev_drv = {0};
static lv_obj_t *arc;
static lv_obj_t *bubble;

void lvgl_display_init() {
    // 初始化LVGL显示驱动
    lv_init();
    
    // 配置显示接口
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_DEFAULT();
    lvgl_port_init(&lvgl_cfg);

    // 初始化显示面板（以ST7789为例）
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    
    // 配置背光控制
    gpio_set_direction((gpio_num_t)PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
}

void create_level_indicator() {
    // 创建主容器
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x000000), 0);

    // 创建水平仪圆形背景
    arc = lv_arc_create(cont);
    lv_obj_set_size(arc, 200, 200);
    lv_obj_center(arc);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_range(arc, -30, 30);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_width(arc, 3, 0);

    // 创建气泡指示器
    bubble = lv_obj_create(cont);
    lv_obj_set_size(bubble, 20, 20);
    lv_obj_set_style_radius(bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(bubble, lv_color_hex(0xFF0000), 0);
    lv_obj_center(bubble);
}

void lvgl_task(void *arg) {
    while (1) {
        // 更新LVGL定时器
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 传感器数据更新接口（需要与传感器任务同步）
void update_level_indicator(float x, float y) {
    // 将加速度数据转换为角度（示例值）
    float angle_x = x * 10.0f;  // 根据实际灵敏度调整
    float angle_y = y * 10.0f;

    // 更新气泡位置
    lv_coord_t pos_x = lv_obj_get_width(arc)/2 + (angle_x * 3);
    lv_coord_t pos_y = lv_obj_get_height(arc)/2 + (angle_y * 3);
    lv_obj_set_pos(bubble, pos_x, pos_y);

    // 更新圆弧指示
    lv_arc_set_value(arc, angle_x);
}
