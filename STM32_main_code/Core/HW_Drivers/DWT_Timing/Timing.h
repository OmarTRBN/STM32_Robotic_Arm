/*
 * Timing.h
 *
 *  Created on: Mar 4, 2025
 *      Author: omart
 */

#ifndef INC_TIMING_H_
#define INC_TIMING_H_

#include "stm32f4xx.h"

void DWT_Init(void);
float DWT_GetDeltaTime(void);

#endif /* INC_TIMING_H_ */
