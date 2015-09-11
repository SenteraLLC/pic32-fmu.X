////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Core Time Management.
////////////////////////////////////////////////////////////////////////////////

#ifndef CORETIME_H_
#define	CORETIME_H_

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <xc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

#define CORETIME_TICKS_PER_SECOND       40000000UL

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of processor ticks.
///
/// @return Number of processor ticks since reset.
///
/// Returns the number of unitless 64-bit core timer tick counts.
////////////////////////////////////////////////////////////////////////////////
uint64_t CoreTimeTicksGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of seconds.
///
/// @return Number of seconds since reset.
///
/// Returns the number of seconds (64-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint64_t CoreTime64sGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of milliseconds.
///
/// @return Number of milliseconds since reset.
///
/// Returns the number of milliseconds (64-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint64_t CoreTime64msGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of microseconds.
///
/// @return Number of microseconds since reset.
///
/// Returns the number of microseconds (64-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint64_t CoreTime64usGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of seconds.
///
/// @return Number of seconds since reset.
///
/// Returns the number of seconds (32-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint32_t CoreTime32sGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of milliseconds.
///
/// @return Number of milliseconds since reset.
///
/// Returns the number of milliseconds (32-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint32_t CoreTime32msGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of microseconds.
///
/// @return Number of microseconds since reset.
///
/// Returns the number of microseconds (32-bit rollover).
////////////////////////////////////////////////////////////////////////////////
uint32_t CoreTime32usGet();

////////////////////////////////////////////////////////////////////////////////
/// @brief  Delay a number of microseconds.
///
/// @param  us
///             Number of microseconds to delay.
///
/// @note   The exact time elapsed will be between 0-1us less that the value
///         supplied.  Therefore, if \p us is '5', the total delay time will
///         be in the range 4-5us.
///
/// This function used the supplied \p us to delay software processing.
////////////////////////////////////////////////////////////////////////////////
void CoreTimeDelayUs(uint32_t us);

////////////////////////////////////////////////////////////////////////////////
/// @brief  Delay a number of milliseconds.
///
/// @param  ms
///             Number of milliseconds to delay.
///
/// @note   The exact time elapsed will be between 0-1ms less that the value
///         supplied.  Therefore, if \p ms is '5', the total delay time will
///         be in the range 4-5ms.
///
/// This function used the supplied \p us to delay software processing.
////////////////////////////////////////////////////////////////////////////////
void CoreTimeDelayMs(uint32_t ms);

#endif	// CORETIME_H_

