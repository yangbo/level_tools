#pragma once
#include "lvgl.h"
#include "lvgl_private.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_ops.h"

void lvgl_display_init();
void create_level_indicator();
void lvgl_task(void *arg);
