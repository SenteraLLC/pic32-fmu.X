/*******************************************************************************
/
/   Filename:   coretime.c
/
*******************************************************************************/

#include <xc.h>
#include <sys/attribs.h>
#include "coretime.h"
#include "stdtypes.h"


static uint32_t coreTimeOverflow;   // Core timer overflow counter.
                                    // Used to provide 64-bit system time.


//==============================================================================

uint64_t CoreTimeTicksGet()
{
    UINT_64 ticks;

    do {
        IEC0SET = _IEC0_CTIE_MASK;              // Enable CT interrupts.
        Nop();
        IEC0CLR = _IEC0_CTIE_MASK;              // Disable CT interrupts.
        ticks.dwords.dword0 = _mfc0(_CP0_COUNT,
                _CP0_COUNT_SELECT);             // Fetch 32-bit CT register.
        ticks.dwords.dword1 =
                coreTimeOverflow;               // Fetch 32-bit CT overflow.
    } while ((IFS0 & _IFS0_CTIF_MASK) != 0);

    IEC0SET = _IEC0_CTIE_MASK;                  // Enable CT interrupts.

    return ticks.val;
}

//==============================================================================

uint64_t CoreTime64sGet()
{
    return CoreTimeTicksGet() / CORETIME_TICKS_PER_SECOND;
}

//==============================================================================

uint64_t CoreTime64msGet()
{
    return CoreTimeTicksGet() / (CORETIME_TICKS_PER_SECOND / 1000);
}

//==============================================================================

uint64_t CoreTime64usGet()
{
    return CoreTimeTicksGet() / (CORETIME_TICKS_PER_SECOND / 1000000);
}

//==============================================================================

uint32_t CoreTime32sGet()
{
    return CoreTimeTicksGet() / CORETIME_TICKS_PER_SECOND;
}

//==============================================================================

uint32_t CoreTime32msGet()
{
    return CoreTimeTicksGet() / (CORETIME_TICKS_PER_SECOND / 1000);
}

//==============================================================================

uint32_t CoreTime32usGet()
{
    return CoreTimeTicksGet() / (CORETIME_TICKS_PER_SECOND / 1000000);
}

//==============================================================================

void CoreTimeDelayUs(uint32_t us)
{
    uint64_t microseconds = CoreTime64usGet() + us;
    while (CoreTime64usGet() < microseconds);
}

//==============================================================================

void CoreTimeDelayMs(uint32_t ms)
{
    uint64_t milliseconds = CoreTime64msGet() + ms;
    while (CoreTime64msGet() < milliseconds);
}

//==============================================================================

// Core Timer ISR

void __ISR (_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{
    coreTimeOverflow++;             // Increment core timer overflow count.
    IFS0CLR = _IFS0_CTIF_MASK;      // Clear core timer interrupt flag.
}

