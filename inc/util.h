////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Utility functions.
////////////////////////////////////////////////////////////////////////////////

#ifndef UTIL_H_
#define	UTIL_H_

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
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Utility function 16-bit CRC calculation.
///
/// @param  data_p
///             Pointer to data for which CRC is calculated.
/// @param  data_len
///             Number of bytes of data for calculation.
/// @param  crc_start
///             The starting CRC value.
///
/// @return The CRC calculated value.
///
/// @note   No exclusive-OR of the CRC beginning or final value is performed
///         by this function.  If required, this must be performed within the
///         calling function.
///
/// @note   This function can be used to calculate the CRC of non-contiguous
///         memory spaces.  After performing the CRC of the first region,
///         the result can be used for \p crc_start when calculating the CRC
///         for the second region.
///
/// This function calculates the CCITT-16 CRC for the given data.
////////////////////////////////////////////////////////////////////////////////
uint16_t utilCRC16( const void* data_p, uint16_t data_len, uint16_t crc_start );

#endif	// UTIL_H_