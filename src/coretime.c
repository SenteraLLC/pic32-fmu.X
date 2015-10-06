////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Core Time Management.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <sys/attribs.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "coretime.h"
#include "stdtypes.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static volatile uint32_t coreTimeOverflow;  // Core timer overflow counter,
                                            // used to provide 64-bit
                                            // system time.

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

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

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Manage core time counter.
///
/// The overflow ISR is used to expand the internal 32-bit counter to 64-bit
/// size.
////////////////////////////////////////////////////////////////////////////////
void __ISR (_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{
    // Re-write the compare register to its maximum value to have core-timer
    // operate as a continuous roll-over counter.  The compare register is 
    // required to be written to clear the CPU asserted interrupt signal.
    _mtc0(_CP0_COMPARE, _CP0_COMPARE_SELECT, 0xFFFFFFFF);
    
    coreTimeOverflow++;             // Increment core timer overflow count.
    IFS0CLR = _IFS0_CTIF_MASK;      // Clear core timer interrupt flag.
}

