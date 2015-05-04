/*******************************************************************************
/
/   Filename:   coretime.h
/
*******************************************************************************/

#ifndef CORETIME_H
#define	CORETIME_H

#include <stdint.h>


// Typedefs ====================================================================

#define CORETIME_TICKS_PER_SECOND       40000000UL



// Function Prototypes =========================================================

uint64_t CoreTimeTicksGet();    // Get unitless 64-bit core timer tick count.

uint64_t CoreTime64sGet();      // Get 64-bit core time in seconds.
uint64_t CoreTime64msGet();     // Get 64-bit core time in milliseconds.
uint64_t CoreTime64usGet();     // Get 64-bit core time in microseconds.

uint32_t CoreTime32sGet();      // Get 32-bit core time in seconds.
uint32_t CoreTime32msGet();     // Get 32-bit core time in milliseconds.
uint32_t CoreTime32usGet();     // Get 32-bit core time in microseconds.

void CoreTimeDelayUs(uint32_t us);
void CoreTimeDelayMs(uint32_t ms);


#endif	/* CORETIME_H */

