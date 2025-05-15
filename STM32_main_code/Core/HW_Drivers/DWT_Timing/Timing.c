/*
 * Timing.c
 *
 *  Created on: Mar 4, 2025
 *      Author: omart
 */

#include "Timing.h"

static uint32_t last_cycle = 0;

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
    DWT->CYCCNT = 0;                                // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           // Start cycle counter
    last_cycle = DWT->CYCCNT;
}

float DWT_GetDeltaTime(void) {
    uint32_t now_cycle = DWT->CYCCNT;
    uint32_t cycle_diff = now_cycle - last_cycle;
    last_cycle = now_cycle;
    return (float)cycle_diff / (float)SystemCoreClock;  // dt in seconds
}

