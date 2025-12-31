#include "driver/adc.hpp"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_adc.h"
#include "py32f071_ll_rcc.h"

#define ADCx ADC1
#define ADC_CHANNEL LL_ADC_CHANNEL_8

static void calib()
{
    LL_ADC_StartCalibration(ADCx);
    while (LL_ADC_IsCalibrationOnGoing(ADCx))
    {
    }
}

void driver::adc::init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PCLK_DIV4);

    // PB0
    do
    {
        LL_GPIO_InitTypeDef init_struct;
        LL_GPIO_StructInit(&init_struct);
        init_struct.Pin = LL_GPIO_PIN_0;
        init_struct.Mode = LL_GPIO_MODE_ANALOG;
        init_struct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOB, &init_struct);
    } while (false);

    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_ADC1);
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_ADC1);

    LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_NONE);
    LL_ADC_SetResolution(ADCx, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADCx, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetSequencersScanMode(ADCx, LL_ADC_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetTriggerSource(ADCx, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADCx, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADCx, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_REG_SetSequencerLength(ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerDiscont(ADCx, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADCx, LL_ADC_REG_RANK_1, ADC_CHANNEL);
    LL_ADC_SetChannelSamplingTime(ADCx, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_41CYCLES_5);

    ::calib();

    LL_ADC_Enable(ADCx);
}

void driver::adc::calib()
{
    LL_ADC_Disable(ADCx);
    ::calib();
    LL_ADC_Enable(ADCx);
}

uint16_t driver::adc::get_voltage()
{
    LL_ADC_REG_StartConversionSWStart(ADCx);
    while (!LL_ADC_IsActiveFlag_EOS(ADCx))
    {
    }

    LL_ADC_ClearFlag_JEOS(ADCx);

    return LL_ADC_REG_ReadConversionData12(ADCx);
}
