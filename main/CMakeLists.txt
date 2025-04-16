idf_component_register(
    SRCS 
        
        "ui/level_indicator.cpp"
    INCLUDE_DIRS 
        "."
        "ui"
    REQUIRES 
        lvgl
        esp_lcd
        driver
)
