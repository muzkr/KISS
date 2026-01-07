
add_library(BL2_lib INTERFACE)
target_include_directories(BL2_lib INTERFACE .)
target_link_libraries(BL2_lib INTERFACE PY32F071_Driver)

target_compile_definitions(BL2_lib INTERFACE
    PY32F071xB
    USE_FULL_LL_DRIVER
)

target_sources(BL2_lib INTERFACE bl2.c)
