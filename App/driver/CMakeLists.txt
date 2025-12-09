
add_library(KISS_Driver INTERFACE)
target_include_directories(KISS_Driver INTERFACE inc)

target_link_libraries(KISS_Driver INTERFACE 
    PY32F071_Driver 
    ASS
    printf
)

target_sources(KISS_Driver INTERFACE
    keypad.cpp
    lcd.cpp
    backlight.cpp
    serial.cpp
    adc.cpp
)
