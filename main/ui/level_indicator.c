#include <math.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "ui/level_indicator.h"
#include "lv_demos.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_touch_cst816s.h"

#include "driver/i2c.h"

/* LCD size */
#define EXAMPLE_LCD_H_RES (240)
#define EXAMPLE_LCD_V_RES (280)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS (8)
#define EXAMPLE_LCD_PARAM_BITS (8)
#define EXAMPLE_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
#define EXAMPLE_LCD_BITS_PER_PIXEL (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_RST (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_DC (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL (GPIO_NUM_15)

#define EXAMPLE_USE_TOUCH 1

#define TOUCH_HOST I2C_NUM_0

#if EXAMPLE_USE_TOUCH
#define EXAMPLE_PIN_NUM_TOUCH_SCL (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_TOUCH_SDA (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_TOUCH_RST (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_TOUCH_INT (GPIO_NUM_14)

esp_lcd_touch_handle_t tp = NULL;
#endif

static const char *TAG = "EXAMPLE";

static lv_obj_t *avatar;

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;

esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);

    return ret;

err:
    if (lcd_panel)
    {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io)
    {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    assert(tp);

    uint16_t tp_x;
    uint16_t tp_y;
    uint8_t tp_cnt = 0;
    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);
    /* Read data from touch controller */
    bool tp_pressed = esp_lcd_touch_get_coordinates(tp, &tp_x, &tp_y, NULL, &tp_cnt, 1);
    if (tp_pressed && tp_cnt > 0)
    {
        data->point.x = tp_x;
        data->point.y = tp_y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", tp_x, tp_y);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 4096,       /* LVGL task stack size */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }};
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    return ESP_OK;
}

static void _app_button_cb(lv_event_t *e)
{
    lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
    rotation++;
    if (rotation > LV_DISPLAY_ROTATION_270)
    {
        rotation = LV_DISPLAY_ROTATION_0;
    }

    /* LCD HW rotation */
    lv_disp_set_rotation(lvgl_disp, rotation);
}

void create_level_indicator();

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(-1);

    // lv_demo_widgets();
    create_level_indicator();

    // LV_IMG_DECLARE(img_test3);
    // avatar = lv_img_create(scr);
    // lv_img_set_src(avatar, &img_test3);

    /* Task unlock */
    lvgl_port_unlock();
}

static lv_obj_t *arc;
static lv_obj_t *bubble;
static lv_obj_t *info_label;
static lv_obj_t *arrow_line; // 新增箭头对象

lv_color_t *buffer = NULL;

// 气泡半径
#define BUBBLE_RADIUS 10

// radius 半径
lv_obj_t *draw_solid_circle(lv_obj_t *parent, int radius)
{
    int buffer_size = radius * 2;
    // 创建一个画布，用于绘制带有透明区域的控件
    buffer = malloc(LV_IMG_BUF_SIZE_TRUE_COLOR_ALPHA(buffer_size,buffer_size));
    lv_obj_t * canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, buffer, buffer_size, buffer_size, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0);

    // 设置画布背景为透明黑色
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_0);

    // 绘制一个圆形，设置为非透明
    lv_draw_rect_dsc_t draw_dsc;
    lv_draw_rect_dsc_init(&draw_dsc);
    draw_dsc.bg_color = lv_palette_main(LV_PALETTE_RED);
    draw_dsc.bg_opa = LV_OPA_COVER;
    draw_dsc.radius = radius;   // 这一句是关键，没有就是矩形，这里设置圆角的半径
    lv_canvas_draw_rect(canvas, 0, 0, radius*2, radius*2, &draw_dsc);

    return canvas;
}

void create_level_indicator()
{
    // 创建主容器
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));                // PCT 是百分比 percent 的意思，即设置为父对象的100%宽度、高度
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x000000), 0);     // 背景色
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);           // 去掉边框
    // 创建水平仪圆形背景
    arc = lv_arc_create(cont);
    lv_obj_set_size(arc, 200, 200);
    lv_obj_center(arc);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_range(arc, -30, 30);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_width(arc, 3, LV_PART_MAIN);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE); // 禁用点击事件

    // 创建气泡指示器
    bubble = draw_solid_circle(cont, BUBBLE_RADIUS);
    // lv_obj_center(bubble);

    // 创建中心位置空心圆
    lv_obj_t *center_circle = lv_btn_create(arc); // 创建一个按钮对象
    lv_obj_center(center_circle);
    lv_obj_set_size(center_circle, 20, 20);                                             // 设置按钮的大小
    lv_obj_set_style_radius(center_circle, LV_RADIUS_CIRCLE, LV_PART_MAIN);             // 设置按钮的圆角半径为圆形
    lv_obj_set_style_bg_opa(center_circle, 0, LV_PART_MAIN);                            // 设置按钮的背景透明度为0
    lv_obj_set_style_border_width(center_circle, 1, LV_PART_MAIN);                      // 设置按钮的边框宽度
    lv_obj_set_style_border_color(center_circle, lv_color_hex(0XFFFFFF), LV_PART_MAIN); // 设置按钮的边框颜色为蓝色

    // 倾斜角度 label
    info_label = lv_label_create(cont);
    lv_label_set_text(info_label, "X: 0\xB0    Y: 0\xB0");
    lv_obj_align_to(info_label, cont, LV_ALIGN_TOP_MID, -lv_obj_get_width(info_label) / 2, 0);
    lv_obj_set_style_text_color(info_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(info_label, &lv_font_montserrat_18, LV_PART_MAIN);

    // 创建箭头线对象
    arrow_line = lv_line_create(arc);
    lv_obj_set_style_line_color(arrow_line, lv_color_white(), 0);
    lv_obj_set_style_line_width(arrow_line, 2, 0);
    lv_obj_set_style_line_rounded(arrow_line, true, 0);
    // 设置箭头对象和父组件一样大小
    lv_obj_set_size(arrow_line, LV_PCT(100), LV_PCT(100));
}

// 传感器数据更新接口（需要与传感器任务同步）
void update_level_indicator(float x, float y)
{
    // 加速度计 X=1，Y=0 时，屏幕正垂直于地面
    // 加速度计 X=-1，Y=0 时，屏幕倒垂直于地面
    // 加速度计 X=0, Y=-1 时，屏幕左上右下垂直于地面
    // 加速度计 X=0, Y=1 时，屏幕左下右上垂直于地面，电路板上的Y轴坐标方向应该是反了

    // 更新气泡位置
    // 采用二次函数增强小值区域的灵敏度：
    // * 新位移量 = y * (2.0 - fabs(y)) * 控件宽度/2
    // * 当y接近0时，(2.0 - |y|) ≈ 2.0，灵敏度加倍
    // * 当y接近±1时，系数回归1.0，保持原有比例
    lv_obj_align(bubble, LV_ALIGN_CENTER,
                // 让气泡中心位于下面坐标位置
                 y * (2.0 - fabs(y)) * lv_obj_get_width(arc) / 2,
                 -x * (2.0 - fabs(x)) * lv_obj_get_height(arc) / 2);
    // 更新倾斜文字信息
    lv_label_set_text_fmt(info_label, "X: %d\xB0    Y: %d\xB0", (int)(y * 90), (int)(x * 90));

    // 更新圆弧指示
    // lv_arc_set_value(arc, angle_x);

    // 更新箭头位置（从中心到气泡）
    // 获取中心圆坐标（与ARC中心一致）
    lv_coord_t center_x = lv_obj_get_width(arc)/2;
    lv_coord_t center_y = lv_obj_get_height(arc)/2;
    // 获取气泡当前位置，以父中心为原点
    lv_coord_t bubble_x_offset = lv_obj_get_x_aligned(bubble);
    lv_coord_t bubble_y_offset = lv_obj_get_y_aligned(bubble);

    // 创建一个静态变量，以便持续存在，否则无法显示
    static lv_point_t line_points[] = {
        {.x = 0, .y = 0}, // 起点
        {.x = 0, .y = 0}  // 终点
    };
    // 更新坐标
    line_points[0].x = center_x;
    line_points[0].y = center_y;
    line_points[1].x = center_x - bubble_x_offset;  // 反向
    line_points[1].y = center_y - bubble_y_offset;

    lv_line_set_points(arrow_line, line_points, 2);
}

void ui_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

#if EXAMPLE_USE_TOUCH
    ESP_LOGI(TAG, "Initialize I2C bus");
    esp_log_level_set("lcd_panel.io.i2c", ESP_LOG_NONE);
    esp_log_level_set("CST816S", ESP_LOG_NONE);
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100 * 1000,
    };
    i2c_param_config(TOUCH_HOST, &i2c_conf);

    i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0);

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // Attach the TOUCH to the I2C bus
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle);

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller");
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);
#endif

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = lvgl_disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Show LVGL objects */
    app_main_display();
}
