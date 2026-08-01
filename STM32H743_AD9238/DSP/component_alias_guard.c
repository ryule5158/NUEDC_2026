#include "component_alias_guard.h"

#include <stddef.h>
#include <string.h>

static float ComponentAliasGuard_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t ComponentAliasGuard_HasReferenceLine(
    const PeriodicAnalyzer_Result *reference,
    float frequency_hz,
    float required_peak_v,
    float frequency_tolerance_hz)
{
    if ((reference == NULL) || (reference->valid == 0U) ||
        (reference->status != PERIODIC_ANALYZER_OK)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < reference->spectrum_component_count; ++i) {
        if ((reference->spectrum[i].peak_amplitude_v >= required_peak_v) &&
            (ComponentAliasGuard_Abs(
                 reference->spectrum[i].frequency_hz - frequency_hz) <=
             frequency_tolerance_hz)) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t ComponentAliasGuard_IsSupported(
    const PeriodicAnalyzer_Result *reference,
    float frequency_hz,
    float primary_peak_v,
    float frequency_tolerance_hz,
    float material_peak_v,
    float reference_min_peak_v,
    float reference_min_ratio)
{
    float required_reference_peak_v;

    /* Lines below 5 mVpp cannot materially affect the required 5 mV result. */
    if (primary_peak_v < material_peak_v) return 1U;

    required_reference_peak_v = primary_peak_v * reference_min_ratio;
    if (required_reference_peak_v < reference_min_peak_v) {
        required_reference_peak_v = reference_min_peak_v;
    }
    return ComponentAliasGuard_HasReferenceLine(reference,
                                                 frequency_hz,
                                                 required_reference_peak_v,
                                                 frequency_tolerance_hz);
}

void ComponentAliasGuard_Apply(
    PeriodicAnalyzer_Result *ch1,
    const PeriodicAnalyzer_Result *ch2,
    float frequency_tolerance_hz,
    float material_peak_v,
    float reference_min_peak_v,
    float reference_min_ratio,
    ComponentAliasGuard_Report *report)
{
    ComponentAliasGuard_Report local_report = {
        COMPONENT_ALIAS_GUARD_KEEP_CH1, 0U
    };
    uint8_t model_supported = 1U;

    if ((ch1 == NULL) || !(frequency_tolerance_hz > 0.0f) ||
        !(material_peak_v > 0.0f) || !(reference_min_peak_v > 0.0f) ||
        !(reference_min_ratio > 0.0f)) {
        if (report != NULL) *report = local_report;
        return;
    }

    if ((ch1->valid == 0U) || (ch1->status != PERIODIC_ANALYZER_OK)) {
        if ((ch2 != NULL) && (ch2->valid != 0U) &&
            (ch2->status == PERIODIC_ANALYZER_OK)) {
            *ch1 = *ch2;
            local_report.mode = COMPONENT_ALIAS_GUARD_USE_CH2;
        }
        if (report != NULL) *report = local_report;
        return;
    }

    for (uint8_t i = 0U; i < ch1->component_count; ++i) {
        if (ComponentAliasGuard_IsSupported(
                ch2,
                ch1->component[i].frequency_hz,
                ch1->component[i].peak_amplitude_v,
                frequency_tolerance_hz,
                material_peak_v,
                reference_min_peak_v,
                reference_min_ratio) == 0U) {
            model_supported = 0U;
            break;
        }
    }

    {
        uint8_t destination = 0U;
        const uint8_t original_count = ch1->spectrum_component_count;
        for (uint8_t source = 0U; source < original_count; ++source) {
            if (ComponentAliasGuard_IsSupported(
                    ch2,
                    ch1->spectrum[source].frequency_hz,
                    ch1->spectrum[source].peak_amplitude_v,
                    frequency_tolerance_hz,
                    material_peak_v,
                    reference_min_peak_v,
                    reference_min_ratio) != 0U) {
                if (destination != source) {
                    ch1->spectrum[destination] = ch1->spectrum[source];
                }
                destination++;
            } else {
                local_report.removed_spectrum_count++;
            }
        }
        ch1->spectrum_component_count = destination;
    }

    if (model_supported == 0U) {
        if ((ch2 != NULL) && (ch2->valid != 0U) &&
            (ch2->status == PERIODIC_ANALYZER_OK)) {
            *ch1 = *ch2;
            local_report.mode = COMPONENT_ALIAS_GUARD_USE_CH2;
        } else {
            memset(ch1, 0, sizeof(*ch1));
            ch1->status = PERIODIC_ANALYZER_E_NO_SIGNAL;
            local_report.mode = COMPONENT_ALIAS_GUARD_REJECT;
        }
    }

    if (report != NULL) *report = local_report;
}
