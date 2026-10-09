# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

#FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

#idf_component_register(SRCS ${app_sources})

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_adc
)
