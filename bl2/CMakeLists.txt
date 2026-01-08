
add_subdirectory(lib)

# -----------------------------
#   BL2

add_library(BL2 INTERFACE)
target_include_directories(BL2 INTERFACE .)
target_link_libraries(BL2 INTERFACE PY32F071_Driver CherryUSB FAT BL2_lib)

target_sources(BL2 INTERFACE
    main.c
    board.c
    py25q16.c
    usbd_msc_impl.c
    fw.c
    py32f071_it.c
    system_py32f071.c
    startup_py32f071xx.s
)

target_compile_definitions(BL2 INTERFACE
    PY32F071xB
    USE_FULL_LL_DRIVER
)

target_link_libraries(BL2 INTERFACE printf)
target_compile_definitions(BL2 INTERFACE PRINTF_INCLUDE_CONFIG_H)

target_link_options(BL2 INTERFACE "-Wl,--script=${CMAKE_CURRENT_SOURCE_DIR}/py32f071xb.ld")
