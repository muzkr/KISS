
add_library(Core INTERFACE)

target_include_directories(Core INTERFACE Inc)

target_sources(Core INTERFACE
    Src/main.c
    Src/py32f071_it.c
    Src/system_py32f071.c
    startup_py32f071xx.s
)

target_link_libraries(Core INTERFACE CMSIS PY32F071_Driver)

target_compile_definitions(Core INTERFACE
    PY32F071xB
    USE_FULL_LL_DRIVER
)

target_link_options(Core BEFORE INTERFACE -T ${CMAKE_CURRENT_SOURCE_DIR}/py32f071xb.ld)
