
add_subdirectory(driver)

# --------------------

add_library(App INTERFACE)
target_include_directories(App INTERFACE .)

target_sources(App INTERFACE
    main.cpp
    py32f071_it.c
    system_py32f071.c
    app_freertos.c
)

target_link_libraries(App INTERFACE 
    PY32F071_Driver 
    ASS
    printf
    KISS_Driver
)

target_compile_definitions(App INTERFACE
    PY32F071xB
    USE_FULL_LL_DRIVER
    PRINTF_INCLUDE_CONFIG_H
)
