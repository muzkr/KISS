
add_library(ASS INTERFACE)
target_include_directories(ASS INTERFACE 
    ass/FreeRTOS/Source/include
    ass/FreeRTOS/Source/portable/GCC/ARM_CM0
)
target_sources(ASS INTERFACE
    ass/FreeRTOS/Source/event_groups.c 
    ass/FreeRTOS/Source/list.c 
    ass/FreeRTOS/Source/queue.c 
    ass/FreeRTOS/Source/stream_buffer.c 
    ass/FreeRTOS/Source/tasks.c 
    ass/FreeRTOS/Source/timers.c 
    ass/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c 
)
