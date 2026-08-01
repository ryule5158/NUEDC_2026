#ifndef COMPONENT_ALIAS_GUARD_H
#define COMPONENT_ALIAS_GUARD_H

#include "periodic_analyzer.h"
#include <stdint.h>

typedef enum {
    COMPONENT_ALIAS_GUARD_KEEP_CH1 = 0,
    COMPONENT_ALIAS_GUARD_USE_CH2 = 1,
    COMPONENT_ALIAS_GUARD_REJECT = 2
} ComponentAliasGuard_Mode;

typedef struct {
    ComponentAliasGuard_Mode mode;
    uint8_t removed_spectrum_count;
} ComponentAliasGuard_Report;

/*
 * Keep CH1 unchanged when all material CH1 model components also exist in
 * CH2. Unmatched CH1-only spectrum lines are removed. If an unmatched line
 * has entered the CH1 harmonic model, use the filtered CH2 result instead.
 */
void ComponentAliasGuard_Apply(
    PeriodicAnalyzer_Result *ch1,
    const PeriodicAnalyzer_Result *ch2,
    float frequency_tolerance_hz,
    float material_peak_v,
    float reference_min_peak_v,
    float reference_min_ratio,
    ComponentAliasGuard_Report *report);

#endif /* COMPONENT_ALIAS_GUARD_H */
