
add_library(printf INTERFACE)
target_include_directories(printf INTERFACE printf)
target_sources(printf INTERFACE printf/printf.c)
