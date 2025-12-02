
add_library(CMSIS INTERFACE)
target_include_directories(CMSIS INTERFACE 
    CMSIS/Include
    CMSIS/Device/PY32F071/Include
)

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
