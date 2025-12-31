
# ------------------------------------
#       CMSIS

add_library(CMSIS INTERFACE)
target_include_directories(CMSIS INTERFACE 
    CMSIS/Include
    CMSIS/Device/PY32F071/Include
)

# ------------------------------------
#       PY32F071_Driver

add_library(PY32F071_Driver INTERFACE)
target_link_libraries(PY32F071_Driver INTERFACE CMSIS)
target_include_directories(PY32F071_Driver INTERFACE
    PY32F071_HAL_Driver/Inc
)
target_sources(PY32F071_Driver INTERFACE
    PY32F071_HAL_Driver/Src/py32f071_ll_rcc.c
    PY32F071_HAL_Driver/Src/py32f071_ll_pwr.c
    PY32F071_HAL_Driver/Src/py32f071_ll_dma.c
    PY32F071_HAL_Driver/Src/py32f071_ll_gpio.c
    PY32F071_HAL_Driver/Src/py32f071_ll_exti.c
    PY32F071_HAL_Driver/Src/py32f071_ll_tim.c
    PY32F071_HAL_Driver/Src/py32f071_ll_usart.c
    PY32F071_HAL_Driver/Src/py32f071_ll_utils.c
    PY32F071_HAL_Driver/Src/py32f071_ll_adc.c
    PY32F071_HAL_Driver/Src/py32f071_ll_dac.c
    PY32F071_HAL_Driver/Src/py32f071_ll_spi.c
    PY32F071_HAL_Driver/Src/py32f071_ll_i2c.c
    PY32F071_HAL_Driver/Src/py32f071_ll_rtc.c
)

# ------------------------------------
#       ASS

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

# ------------------------------------
#       CherryUSB

add_library(CherryUSB INTERFACE)
target_include_directories(CherryUSB INTERFACE
    CherryUSB/port
    CherryUSB/core
    CherryUSB/common
    CherryUSB/class/msc
)
target_sources(CherryUSB INTERFACE
    CherryUSB/core/usbd_core.c
    CherryUSB/port/usb_dc_py32.c
    CherryUSB/class/msc/usbd_msc.c
)

# ------------------------------------
#       printf

add_library(printf INTERFACE)
target_include_directories(printf INTERFACE printf)
target_sources(printf INTERFACE printf/printf.c)
