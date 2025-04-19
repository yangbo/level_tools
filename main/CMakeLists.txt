#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(
    SRCS 
        "main.c"
        "ui/level_indicator.c"
        ${SRC_UI}
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
