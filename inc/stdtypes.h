////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Standard type definitions.
////////////////////////////////////////////////////////////////////////////////

#ifndef STDTYPES_H_
#define STDTYPES_H_

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

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

typedef union u8bit {           // Unsigned 8-bit
    uint8_t val;
    struct {
        uint8_t bit0 : 1;
        uint8_t bit1 : 1;
        uint8_t bit2 : 1;
        uint8_t bit3 : 1;
        uint8_t bit4 : 1;
        uint8_t bit5 : 1;
        uint8_t bit6 : 1;
        uint8_t bit7 : 1;
    } bits;
} UINT_8;

typedef union s8bit {           // Signed 8-bit
    int8_t val;
    struct {
        uint8_t bit0 : 1;
        uint8_t bit1 : 1;
        uint8_t bit2 : 1;
        uint8_t bit3 : 1;
        uint8_t bit4 : 1;
        uint8_t bit5 : 1;
        uint8_t bit6 : 1;
        uint8_t bit7 : 1;
    } bits;
} INT_8;

typedef union u16bit {          // Unsigned 16-bit
    uint16_t val;
    struct {
        uint8_t byte0;
        uint8_t byte1;
    } bytes;
    struct {
        uint16_t bit0 : 1;
        uint16_t bit1 : 1;
        uint16_t bit2 : 1;
        uint16_t bit3 : 1;
        uint16_t bit4 : 1;
        uint16_t bit5 : 1;
        uint16_t bit6 : 1;
        uint16_t bit7 : 1;
        uint16_t bit8 : 1;
        uint16_t bit9 : 1;
        uint16_t bit10 : 1;
        uint16_t bit11 : 1;
        uint16_t bit12 : 1;
        uint16_t bit13 : 1;
        uint16_t bit14 : 1;
        uint16_t bit15 : 1;
    } bits;
} UINT_16;

typedef union s16bit {          // Signed 16-bit
    int16_t val;
    struct {
        uint8_t byte0;
        uint8_t byte1;
    } bytes;
    struct {
        uint16_t bit0 : 1;
        uint16_t bit1 : 1;
        uint16_t bit2 : 1;
        uint16_t bit3 : 1;
        uint16_t bit4 : 1;
        uint16_t bit5 : 1;
        uint16_t bit6 : 1;
        uint16_t bit7 : 1;
        uint16_t bit8 : 1;
        uint16_t bit9 : 1;
        uint16_t bit10 : 1;
        uint16_t bit11 : 1;
        uint16_t bit12 : 1;
        uint16_t bit13 : 1;
        uint16_t bit14 : 1;
        uint16_t bit15 : 1;
    } bits;
} INT_16;

typedef union u24bit {          // Unsigned 24-bit
    uint32_t val : 24;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        unsigned :8;
    } bytes;
    struct {
        uint32_t bit0 : 1;
        uint32_t bit1 : 1;
        uint32_t bit2 : 1;
        uint32_t bit3 : 1;
        uint32_t bit4 : 1;
        uint32_t bit5 : 1;
        uint32_t bit6 : 1;
        uint32_t bit7 : 1;
        uint32_t bit8 : 1;
        uint32_t bit9 : 1;
        uint32_t bit10 : 1;
        uint32_t bit11 : 1;
        uint32_t bit12 : 1;
        uint32_t bit13 : 1;
        uint32_t bit14 : 1;
        uint32_t bit15 : 1;
        uint32_t bit16 : 1;
        uint32_t bit17 : 1;
        uint32_t bit18 : 1;
        uint32_t bit19 : 1;
        uint32_t bit20 : 1;
        uint32_t bit21 : 1;
        uint32_t bit22 : 1;
        uint32_t bit23 : 1;
        unsigned :8;
    } bits;
} UINT_24;

typedef union s24bit {          // Signed 24-bit
    int32_t val : 24;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
    } bytes;
    struct {
        uint32_t bit0 : 1;
        uint32_t bit1 : 1;
        uint32_t bit2 : 1;
        uint32_t bit3 : 1;
        uint32_t bit4 : 1;
        uint32_t bit5 : 1;
        uint32_t bit6 : 1;
        uint32_t bit7 : 1;
        uint32_t bit8 : 1;
        uint32_t bit9 : 1;
        uint32_t bit10 : 1;
        uint32_t bit11 : 1;
        uint32_t bit12 : 1;
        uint32_t bit13 : 1;
        uint32_t bit14 : 1;
        uint32_t bit15 : 1;
        uint32_t bit16 : 1;
        uint32_t bit17 : 1;
        uint32_t bit18 : 1;
        uint32_t bit19 : 1;
        uint32_t bit20 : 1;
        uint32_t bit21 : 1;
        uint32_t bit22 : 1;
        uint32_t bit23 : 1;
    } bits;
} INT_24;

typedef union u32bit {          // Unsigned 32-bit
    uint32_t val;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    } bytes;
    struct {
        uint32_t bit0 : 1;
        uint32_t bit1 : 1;
        uint32_t bit2 : 1;
        uint32_t bit3 : 1;
        uint32_t bit4 : 1;
        uint32_t bit5 : 1;
        uint32_t bit6 : 1;
        uint32_t bit7 : 1;
        uint32_t bit8 : 1;
        uint32_t bit9 : 1;
        uint32_t bit10 : 1;
        uint32_t bit11 : 1;
        uint32_t bit12 : 1;
        uint32_t bit13 : 1;
        uint32_t bit14 : 1;
        uint32_t bit15 : 1;
        uint32_t bit16 : 1;
        uint32_t bit17 : 1;
        uint32_t bit18 : 1;
        uint32_t bit19 : 1;
        uint32_t bit20 : 1;
        uint32_t bit21 : 1;
        uint32_t bit22 : 1;
        uint32_t bit23 : 1;
        uint32_t bit24 : 1;
        uint32_t bit25 : 1;
        uint32_t bit26 : 1;
        uint32_t bit27 : 1;
        uint32_t bit28 : 1;
        uint32_t bit29 : 1;
        uint32_t bit30 : 1;
        uint32_t bit31 : 1;
    } bits;
} UINT_32;

typedef union s32bit {          // Signed 32-bit
    int32_t val;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    } bytes;
    struct {
        uint32_t bit0 : 1;
        uint32_t bit1 : 1;
        uint32_t bit2 : 1;
        uint32_t bit3 : 1;
        uint32_t bit4 : 1;
        uint32_t bit5 : 1;
        uint32_t bit6 : 1;
        uint32_t bit7 : 1;
        uint32_t bit8 : 1;
        uint32_t bit9 : 1;
        uint32_t bit10 : 1;
        uint32_t bit11 : 1;
        uint32_t bit12 : 1;
        uint32_t bit13 : 1;
        uint32_t bit14 : 1;
        uint32_t bit15 : 1;
        uint32_t bit16 : 1;
        uint32_t bit17 : 1;
        uint32_t bit18 : 1;
        uint32_t bit19 : 1;
        uint32_t bit20 : 1;
        uint32_t bit21 : 1;
        uint32_t bit22 : 1;
        uint32_t bit23 : 1;
        uint32_t bit24 : 1;
        uint32_t bit25 : 1;
        uint32_t bit26 : 1;
        uint32_t bit27 : 1;
        uint32_t bit28 : 1;
        uint32_t bit29 : 1;
        uint32_t bit30 : 1;
        uint32_t bit31 : 1;
    } bits;
} INT_32;

typedef union u64bit {          // Unsigned 64-bit
    uint64_t val;
    struct {
        uint32_t dword0;
        uint32_t dword1;
    } dwords;
    struct {
        uint16_t word0;
        uint16_t word1;
        uint16_t word3;
        uint16_t word4;
    } words;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    } bytes;
    struct {
        uint64_t bit0 : 1;
        uint64_t bit1 : 1;
        uint64_t bit2 : 1;
        uint64_t bit3 : 1;
        uint64_t bit4 : 1;
        uint64_t bit5 : 1;
        uint64_t bit6 : 1;
        uint64_t bit7 : 1;
        uint64_t bit8 : 1;
        uint64_t bit9 : 1;
        uint64_t bit10 : 1;
        uint64_t bit11 : 1;
        uint64_t bit12 : 1;
        uint64_t bit13 : 1;
        uint64_t bit14 : 1;
        uint64_t bit15 : 1;
        uint64_t bit16 : 1;
        uint64_t bit17 : 1;
        uint64_t bit18 : 1;
        uint64_t bit19 : 1;
        uint64_t bit20 : 1;
        uint64_t bit21 : 1;
        uint64_t bit22 : 1;
        uint64_t bit23 : 1;
        uint64_t bit24 : 1;
        uint64_t bit25 : 1;
        uint64_t bit26 : 1;
        uint64_t bit27 : 1;
        uint64_t bit28 : 1;
        uint64_t bit29 : 1;
        uint64_t bit30 : 1;
        uint64_t bit31 : 1;
        uint64_t bit32 : 1;
        uint64_t bit33 : 1;
        uint64_t bit34 : 1;
        uint64_t bit35 : 1;
        uint64_t bit36 : 1;
        uint64_t bit37 : 1;
        uint64_t bit38 : 1;
        uint64_t bit39 : 1;
        uint64_t bit40 : 1;
        uint64_t bit41 : 1;
        uint64_t bit42 : 1;
        uint64_t bit43 : 1;
        uint64_t bit44 : 1;
        uint64_t bit45 : 1;
        uint64_t bit46 : 1;
        uint64_t bit47 : 1;
        uint64_t bit48 : 1;
        uint64_t bit49 : 1;
        uint64_t bit50 : 1;
        uint64_t bit51 : 1;
        uint64_t bit52 : 1;
        uint64_t bit53 : 1;
        uint64_t bit54 : 1;
        uint64_t bit55 : 1;
        uint64_t bit56 : 1;
        uint64_t bit57 : 1;
        uint64_t bit58 : 1;
        uint64_t bit59 : 1;
        uint64_t bit60 : 1;
        uint64_t bit61 : 1;
    } bits;
} UINT_64;

typedef union s64bit {          // Signed 64-bit
    int64_t val;
    struct {
        uint32_t dword0;
        uint32_t dword1;
    } dwords;
    struct {
        uint16_t word0;
        uint16_t word1;
        uint16_t word3;
        uint16_t word4;
    } words;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    } bytes;
    struct {
        uint64_t bit0 : 1;
        uint64_t bit1 : 1;
        uint64_t bit2 : 1;
        uint64_t bit3 : 1;
        uint64_t bit4 : 1;
        uint64_t bit5 : 1;
        uint64_t bit6 : 1;
        uint64_t bit7 : 1;
        uint64_t bit8 : 1;
        uint64_t bit9 : 1;
        uint64_t bit10 : 1;
        uint64_t bit11 : 1;
        uint64_t bit12 : 1;
        uint64_t bit13 : 1;
        uint64_t bit14 : 1;
        uint64_t bit15 : 1;
        uint64_t bit16 : 1;
        uint64_t bit17 : 1;
        uint64_t bit18 : 1;
        uint64_t bit19 : 1;
        uint64_t bit20 : 1;
        uint64_t bit21 : 1;
        uint64_t bit22 : 1;
        uint64_t bit23 : 1;
        uint64_t bit24 : 1;
        uint64_t bit25 : 1;
        uint64_t bit26 : 1;
        uint64_t bit27 : 1;
        uint64_t bit28 : 1;
        uint64_t bit29 : 1;
        uint64_t bit30 : 1;
        uint64_t bit31 : 1;
        uint64_t bit32 : 1;
        uint64_t bit33 : 1;
        uint64_t bit34 : 1;
        uint64_t bit35 : 1;
        uint64_t bit36 : 1;
        uint64_t bit37 : 1;
        uint64_t bit38 : 1;
        uint64_t bit39 : 1;
        uint64_t bit40 : 1;
        uint64_t bit41 : 1;
        uint64_t bit42 : 1;
        uint64_t bit43 : 1;
        uint64_t bit44 : 1;
        uint64_t bit45 : 1;
        uint64_t bit46 : 1;
        uint64_t bit47 : 1;
        uint64_t bit48 : 1;
        uint64_t bit49 : 1;
        uint64_t bit50 : 1;
        uint64_t bit51 : 1;
        uint64_t bit52 : 1;
        uint64_t bit53 : 1;
        uint64_t bit54 : 1;
        uint64_t bit55 : 1;
        uint64_t bit56 : 1;
        uint64_t bit57 : 1;
        uint64_t bit58 : 1;
        uint64_t bit59 : 1;
        uint64_t bit60 : 1;
        uint64_t bit61 : 1;
    } bits;
} INT_64;


typedef union fp16bit {         // 16-bit Floating Point
    struct {
        uint8_t byte0;
        uint8_t byte1;
    } bytes;
    struct {
        unsigned fraction : 10;
        unsigned exponent : 5;
        unsigned sign : 1;
    } binary16;
} FP_16;

typedef union fp32bit {         // 32-bit Floating Point
    float val;
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    } bytes;
    struct {
        unsigned fraction : 23;
        unsigned exponent : 8;
        unsigned sign : 1;
    } binary32;
} FP_32;


// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

#endif // STDTYPES_H_
