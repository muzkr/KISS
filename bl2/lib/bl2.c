
#include "bl2.h"
#include "py32f071_ll_pwr.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_rcc.h"
#include "py32f071_ll_rtc.h"

#define BL2_MARK 0xbb

static bool extract_boot_mode(uint32_t PRL, uint32_t ALR, uint8_t *boot_mode, uint32_t *param)
{
    if (BL2_MARK != (PRL >> 8))
    {
        return false;
    }

    *boot_mode = 0xff & PRL;
    *param = ALR;
    return true;
}

static inline void make_boot_mode(uint32_t *PRL, uint32_t *ALR, uint8_t boot_mode, uint32_t param)
{
    *PRL = (BL2_MARK << 8) | boot_mode;
    *ALR = param;
}

bool bl2_get_boot_mode(uint8_t *boot_mode, uint32_t *param)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_EnableBkUpAccess();

    if (LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI || !LL_RCC_IsEnabledRTC())
    {
        return false;
    }

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);

    // LL_RTC_ClearFlag_RS(RTC);
    // while (!LL_RTC_IsActiveFlag_RS(RTC))
    // {
    // }

    uint32_t ALR = (RTC->ALRH << 16) | (0xffff & RTC->ALRL);
    uint32_t PRL = (RTC->PRLH << 16) | (0xffff & RTC->PRLL);

    LL_RCC_DisableRTC();
    // LL_RCC_ForceBackupDomainReset();
    // LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_LSI_Disable();
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_RTC);

    return extract_boot_mode(PRL, ALR, boot_mode, param);
    // return true;
}

void bl2_set_boot_mode(uint8_t boot_mode, uint32_t param)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_EnableBkUpAccess();

    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1)
    {
    }

    if (LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
    {
        LL_RCC_ForceBackupDomainReset();
        LL_RCC_ReleaseBackupDomainReset();
        LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
    }

    /* Enable RTC clock and RTC APB clock*/
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);
    LL_RCC_EnableRTC();

    LL_RTC_ClearFlag_RS(RTC);
    while (!LL_RTC_IsActiveFlag_RS(RTC))
    {
    }

    /* Disable the write protection for RTC registers */
    LL_RTC_DisableWriteProtection(RTC);

    while (!LL_RTC_IsActiveFlag_RTOF(RTC))
    {
    }

    uint32_t PRL, ALR;
    make_boot_mode(&PRL, &ALR, boot_mode, param);
    LL_RTC_ALARM_Set(RTC, ALR);
    LL_RTC_SetAsynchPrescaler(RTC, PRL);

    LL_RTC_EnableWriteProtection(RTC);

    while (!LL_RTC_IsActiveFlag_RTOF(RTC))
    {
    }
}
