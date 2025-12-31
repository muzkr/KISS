
add_subdirectory(driver)

# --------------------

add_library(App INTERFACE)
target_include_directories(App INTERFACE . usb)

target_sources(App INTERFACE
    blink_task.cpp
    main.c
    main_task.cpp
    py32f071_it.c
    system_py32f071.c
    app_freertos.c
    startup_py32f071xx.s
)

target_link_libraries(App INTERFACE 
    KISS_Driver
    PY32F071_Driver 
    ASS
    printf
)

target_compile_definitions(App INTERFACE
    PY32F071xB
    USE_FULL_LL_DRIVER
    PRINTF_INCLUDE_CONFIG_H
)

target_link_options(App INTERFACE "-Wl,--script=${CMAKE_CURRENT_SOURCE_DIR}/py32f071xb.ld")
