idf_component_register(
    SRCS 
        "main.c"
        "ui/level_indicator.c"
    INCLUDE_DIRS 
        "."
        "ui"
        "../components/SensorLib/src"
    REQUIRES 
        lvgl
        esp_lcd
        driver
)

set_source_files_properties(main.c PROPERTIES LANGUAGE CXX)
