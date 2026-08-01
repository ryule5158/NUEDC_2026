#include "component_alias_guard.h"

#include <stdio.h>
#include <string.h>

static void SetValid(PeriodicAnalyzer_Result *result)
{
    memset(result, 0, sizeof(*result));
    result->status = PERIODIC_ANALYZER_OK;
    result->valid = 1U;
}

static void AddModel(PeriodicAnalyzer_Result *result,
                     unsigned index,
                     float frequency_hz,
                     float peak_v)
{
    result->component[index].harmonic_order = (uint16_t)(index + 1U);
    result->component[index].frequency_hz = frequency_hz;
    result->component[index].peak_amplitude_v = peak_v;
    result->component_count = (uint8_t)(index + 1U);
}

static void AddSpectrum(PeriodicAnalyzer_Result *result,
                        unsigned index,
                        float frequency_hz,
                        float peak_v)
{
    result->spectrum[index].frequency_hz = frequency_hz;
    result->spectrum[index].peak_amplitude_v = peak_v;
    result->spectrum_component_count = (uint8_t)(index + 1U);
}

static int TestKeepCh1AndRemoveAlias(void)
{
    PeriodicAnalyzer_Result ch1;
    PeriodicAnalyzer_Result ch2;
    ComponentAliasGuard_Report report;

    SetValid(&ch1);
    SetValid(&ch2);
    ch1.vpp_v = 0.123456f;
    AddModel(&ch1, 0U, 100000.0f, 0.025f);
    AddModel(&ch1, 1U, 200000.0f, 0.012f);
    AddSpectrum(&ch1, 0U, 100000.0f, 0.025f);
    AddSpectrum(&ch1, 1U, 200000.0f, 0.012f);
    AddSpectrum(&ch1, 2U, 350000.0f, 0.100f);
    AddSpectrum(&ch2, 0U, 100100.0f, 0.024f);
    AddSpectrum(&ch2, 1U, 199900.0f, 0.011f);

    ComponentAliasGuard_Apply(&ch1, &ch2, 750.0f, 0.0025f,
                              0.0005f, 0.20f, &report);
    if ((report.mode != COMPONENT_ALIAS_GUARD_KEEP_CH1) ||
        (report.removed_spectrum_count != 1U) ||
        (ch1.spectrum_component_count != 2U) ||
        (ch1.vpp_v != 0.123456f)) {
        return 1;
    }
    return 0;
}

static int TestFallbackToCh2(void)
{
    PeriodicAnalyzer_Result ch1;
    PeriodicAnalyzer_Result ch2;
    ComponentAliasGuard_Report report;

    SetValid(&ch1);
    SetValid(&ch2);
    ch1.vpp_v = 0.200f;
    ch2.vpp_v = 0.050f;
    AddModel(&ch1, 0U, 350000.0f, 0.100f);
    AddSpectrum(&ch1, 0U, 100000.0f, 0.025f);
    AddSpectrum(&ch1, 1U, 350000.0f, 0.100f);
    AddModel(&ch2, 0U, 100000.0f, 0.025f);
    AddSpectrum(&ch2, 0U, 100000.0f, 0.025f);

    ComponentAliasGuard_Apply(&ch1, &ch2, 750.0f, 0.0025f,
                              0.0005f, 0.20f, &report);
    if ((report.mode != COMPONENT_ALIAS_GUARD_USE_CH2) ||
        (ch1.vpp_v != 0.050f) ||
        (ch1.fundamental_hz != ch2.fundamental_hz)) {
        return 1;
    }
    return 0;
}

static int TestRejectPureAlias(void)
{
    PeriodicAnalyzer_Result ch1;
    PeriodicAnalyzer_Result ch2;
    ComponentAliasGuard_Report report;

    SetValid(&ch1);
    memset(&ch2, 0, sizeof(ch2));
    ch2.status = PERIODIC_ANALYZER_E_NO_SIGNAL;
    AddModel(&ch1, 0U, 200000.0f, 0.100f);
    AddSpectrum(&ch1, 0U, 200000.0f, 0.100f);

    ComponentAliasGuard_Apply(&ch1, &ch2, 750.0f, 0.0025f,
                              0.0005f, 0.20f, &report);
    if ((report.mode != COMPONENT_ALIAS_GUARD_REJECT) ||
        (ch1.valid != 0U) ||
        (ch1.status != PERIODIC_ANALYZER_E_NO_SIGNAL)) {
        return 1;
    }
    return 0;
}

static int TestKeepSubToleranceLine(void)
{
    PeriodicAnalyzer_Result ch1;
    PeriodicAnalyzer_Result ch2;
    ComponentAliasGuard_Report report;

    SetValid(&ch1);
    SetValid(&ch2);
    AddModel(&ch1, 0U, 100000.0f, 0.025f);
    AddSpectrum(&ch1, 0U, 100000.0f, 0.025f);
    AddSpectrum(&ch1, 1U, 320000.0f, 0.001f);
    AddModel(&ch2, 0U, 100000.0f, 0.024f);
    AddSpectrum(&ch2, 0U, 100000.0f, 0.024f);

    ComponentAliasGuard_Apply(&ch1, &ch2, 750.0f, 0.0025f,
                              0.0005f, 0.20f, &report);
    if ((report.mode != COMPONENT_ALIAS_GUARD_KEEP_CH1) ||
        (report.removed_spectrum_count != 0U) ||
        (ch1.spectrum_component_count != 2U)) {
        return 1;
    }
    return 0;
}

int main(void)
{
    if (TestKeepCh1AndRemoveAlias() != 0) return 1;
    if (TestFallbackToCh2() != 0) return 2;
    if (TestRejectPureAlias() != 0) return 3;
    if (TestKeepSubToleranceLine() != 0) return 4;
    puts("COMPONENT_ALIAS_GUARD_TEST_PASS");
    return 0;
}
