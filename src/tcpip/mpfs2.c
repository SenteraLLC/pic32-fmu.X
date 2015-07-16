/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    mpfs2.c

  Summary:
    Module for Microchip TCP/IP Stack
    -Provides single API for accessing web pages and other files
     from internal program memory or an external serial EEPROM memory
    -Reference: AN833

  Description:
    Microchip File System (MPFS) File Access API

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) <2014> released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#define __MPFS2_C_

#include "tcpip/tcpip.h"

#if defined(STACK_USE_MPFS2)

//Supports long file names to 64 characters
#define MAX_FILE_NAME_LEN   (64u)

/*
 * MPFS Structure:
 *     [M][P][F][S]
 *     [uint8_t Ver Hi][uint8_t Ver Lo][uint16_t Number of Files]
 *     [Name Hash 0][Name Hash 1]...[Name Hash N]
 *     [File Record 0][File Record 1]...[File Record N]
 *     [String 0][String 1]...[String N]
 *     [File Data 0][File Data 1]...[File Data N]
 *
 * Name Hash (2 bytes):
 *     hash = 0
 *     for each(byte in name)
 *         hash += byte
 *         hash <<= 1
 *
 *     Technically this means the hash only includes the
 *     final 15 characters of a name.
 *
 * File Record Structure (22 bytes):
 *     [uint32_t String Ptr][uint32_t Data Ptr]
 *     [uint32_t Len][uint32_t Timestamp][uint32_t Microtime]
 *     [uint16_t Flags]
 *
 *     Pointers are absolute addresses within the MPFS image.
 *     Timestamp is the UNIX timestamp
 *     Microtime is currently unimplemented
 *
 * String Structure (1 to 64 bytes):
 *     ["path/to/file.ext"][0x00]
 *
 * File Data Structure (arbitrary length):
 *      [File Data]
 *
 * Unlike previous versions, there are no delimiters.
 *
 * Name hash is calculated as follows:
 *      hash = 0
 *      for each(byte in name)
 *          hash += byte, hash <<= 1
 *
 * When a file has an index, that index file has no file name,
 * but is accessible as the file immediately following in the image.
 *
 * Current version is 2.1
 */

/****************************************************************************
  Section:
    Module-Only Globals and Functions
  ***************************************************************************/

// Track the MPFS File Handles
// MPFSStubs[0] is reserved for internal use (FAT access)
static MPFS_STUB MPFSStubs[MAX_MPFS_HANDLES+1];

// Allows the MPFS to be locked, preventing access during updates
static bool isMPFSLocked;

// FAT record cache
static MPFS_FAT_RECORD fatCache;

// ID of currently loaded fatCache
static uint16_t fatCacheID;

// Number of files in this MPFS image
static uint16_t numFiles;


static void _LoadFATRecord(uint16_t fatID);
static void _Validate(void);

/****************************************************************************
  Section:
    EEPROM vs Flash Storage Settings
  ***************************************************************************/

#if defined(MPFS_USE_EEPROM)

    // Beginning address of MPFS Image
    #define MPFS_HEAD       MPFS_RESERVE_BLOCK

    // Track the last read address to prevent unnecessary
    // data overhead to switch locations.
    MPFS_PTR lastRead;


#elif defined(MPFS_USE_SPI_FLASH)

    // Beginning address of MPFS Image
    #define MPFS_HEAD       MPFS_RESERVE_BLOCK

#else

    // An address where MPFS data starts in program memory.
    #if defined(__XC8) || defined(__XC32)
        extern ROM uint8_t MPFS_Start[];
        #define MPFS_HEAD       ((uint32_t)(&MPFS_Start[0]))
    #else
        extern uint32_t MPFS_Start;
        #define MPFS_HEAD       MPFS_Start;
    #endif

#endif

/****************************************************************************
  Section:
    Stack-Level Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    void MPFSInit(void)

  Summary:
    Initializes the MPFS module.

  Description:
    Sets all MPFS handles to closed, and initializes access to the EEPROM
    if necessary.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    This function is called only one during lifetime of the application.
  ***************************************************************************/
void MPFSInit(void)
{
    uint8_t i;

    for(i = 1; i <= MAX_MPFS_HANDLES; i++)
    {
        MPFSStubs[i].addr = MPFS_INVALID;
    }

    #if defined(MPFS_USE_EEPROM)
    // Initialize the EEPROM access routines.
    XEEInit();
    lastRead = MPFS_INVALID;
    #endif

    #if defined(MPFS_USE_SPI_FLASH)
    // Initialize SPI Flash access routines.
    SPIFlashInit();
    #endif

    // Validate the image and load numFiles
    _Validate();

    isMPFSLocked = false;

}

/****************************************************************************
  Section:
    Handle Management Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    MPFS_HANDLE MPFSOpen(uint8_t* cFile)

  Description:
    Opens a file in the MPFS2 file system.

  Precondition:
    MPFSInit has been called

  Parameters:
    cFile - a null terminated file name to open

  Returns:
    An MPFS_HANDLE to the opened file if found, or MPFS_INVALID_HANDLE
    if the file could not be found or no free handles exist.
  ***************************************************************************/
MPFS_HANDLE MPFSOpen(uint8_t* cFile)
{
    MPFS_HANDLE hMPFS;
    uint16_t nameHash, i;
    uint16_t hashCache[8];
    uint8_t *ptr, c;

    // Initialize c to avoid "may be used uninitialized" compiler warning
    c = 0;

    // Make sure MPFS is unlocked and we got a filename
    if(*cFile == '\0' || isMPFSLocked == true)
        return MPFS_INVALID_HANDLE;

    // Calculate the name hash to speed up searching
    for(nameHash = 0, ptr = cFile; *ptr != '\0'; ptr++)
    {
        nameHash += *ptr;
        nameHash <<= 1;
    }

    // Find a free file handle to use
    for(hMPFS = 1; hMPFS <= MAX_MPFS_HANDLES; hMPFS++)
        if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
            break;
    if(hMPFS == MAX_MPFS_HANDLES)
        return MPFS_INVALID_HANDLE;

    // Read in hashes, and check remainder on a match.  Store 8 in cache for performance
    for(i = 0; i < numFiles; i++)
    {
        // For new block of 8, read in data
        if((i & 0x07) == 0u)
        {
            MPFSStubs[0].addr = 8 + i*2;
            MPFSStubs[0].bytesRem = 16;
            MPFSGetArray(0, (uint8_t*)hashCache, 16);
        }

        // If the hash matches, compare the full filename
        if(hashCache[i&0x07] == nameHash)
        {
            _LoadFATRecord(i);
            MPFSStubs[0].addr = fatCache.string;
            MPFSStubs[0].bytesRem = 255;

            // Loop over filename to perform comparison
            for(ptr = cFile; *ptr != '\0'; ptr++)
            {
                MPFSGet(0, &c);
                if(*ptr != c)
                    break;
            }

            MPFSGet(0, &c);

            if(c == '\0' && *ptr == '\0')
            {// Filename matches, so return true
                MPFSStubs[hMPFS].addr = fatCache.data;
                MPFSStubs[hMPFS].bytesRem = fatCache.len;
                MPFSStubs[hMPFS].fatID = i;
                return hMPFS;
            }
        }
    }

    // No file name matched, so return nothing
    return MPFS_INVALID_HANDLE;
}

/*****************************************************************************
  Function:
    MPFS_HANDLE MPFSOpenROM(ROM uint8_t* cFile)

  Description:
    Opens a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    cFile - a null terminated file name to open

  Returns:
    An MPFS_HANDLE to the opened file if found, or MPFS_INVALID_HANDLE
    if the file could not be found or no free handles exist.

  Remarks:
    This function is aliased to MPFSOpen on non-PIC18 platforms.
  ***************************************************************************/
#if defined(__XC8)
MPFS_HANDLE MPFSOpenROM(ROM uint8_t* cFile)
{
    MPFS_HANDLE hMPFS;
    uint16_t nameHash, i;
    uint16_t hashCache[8];
    ROM uint8_t *ptr;
    uint8_t c;

    // Make sure MPFS is unlocked and we got a filename
    if(*cFile == '\0' || isMPFSLocked == true)
        return MPFS_INVALID_HANDLE;

    // Calculate the name hash to speed up searching
    for(nameHash = 0, ptr = cFile; *ptr != '\0'; ptr++)
    {
        nameHash += *ptr;
        nameHash <<= 1;
    }

    // Find a free file handle to use
    for(hMPFS = 1; hMPFS <= MAX_MPFS_HANDLES; hMPFS++)
        if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
            break;
    if(hMPFS == MAX_MPFS_HANDLES)
        return MPFS_INVALID_HANDLE;

    // Read in hashes, and check remainder on a match.  Store 8 in cache for performance
    for(i = 0; i < numFiles; i++)
    {
        // For new block of 8, read in data
        if((i & 0x07) == 0u)
        {
            MPFSStubs[0].addr = 8 + i*2;
            MPFSStubs[0].bytesRem = 16;
            MPFSGetArray(0, (uint8_t*)hashCache, 16);
        }

        // If the hash matches, compare the full filename
        if(hashCache[i&0x07] == nameHash)
        {
            _LoadFATRecord(i);
            MPFSStubs[0].addr = fatCache.string;
            MPFSStubs[0].bytesRem = 255;

            // Loop over filename to perform comparison
            for(ptr = cFile; *ptr != '\0'; ptr++)
            {
                MPFSGet(0, &c);
                if(*ptr != c)
                    break;
            }

            MPFSGet(0, &c);

            if(c == '\0' && *ptr == '\0')
            {// Filename matches, so return true
                MPFSStubs[hMPFS].addr = fatCache.data;
                MPFSStubs[hMPFS].bytesRem = fatCache.len;
                MPFSStubs[hMPFS].fatID = i;
                return hMPFS;
            }
        }
    }

    // No file name matched, so return nothing
    return MPFS_INVALID_HANDLE;
}
#endif

/*****************************************************************************
  Function:
    MPFS_HANDLE MPFSOpenID(uint16_t hFatID)

  Summary:
    Quickly re-opens a file.

  Description:
    Quickly re-opens a file in the MPFS2 file system.  Use this function
    along with MPFSGetID() to quickly re-open a file without tying up
    a permanent MPFSStub.

  Precondition:
    None

  Parameters:
    hFatID - the ID of a previous opened file in the FAT

  Returns:
    An MPFS_HANDLE to the opened file if found, or MPFS_INVALID_HANDLE
    if the file could not be found or no free handles exist.
  ***************************************************************************/
MPFS_HANDLE MPFSOpenID(uint16_t hFatID)
{
    MPFS_HANDLE hMPFS;

    // Make sure MPFS is unlocked and we got a valid id
    if(isMPFSLocked == true || hFatID > numFiles)
        return MPFS_INVALID_HANDLE;

    // Find a free file handle to use
    for(hMPFS = 1; hMPFS <= MAX_MPFS_HANDLES; hMPFS++)
        if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
            break;
    if(hMPFS == MAX_MPFS_HANDLES)
        return MPFS_INVALID_HANDLE;

    // Load the FAT record
    _LoadFATRecord(hFatID);

    // Set up the file handle
    MPFSStubs[hMPFS].fatID = hFatID;
    MPFSStubs[hMPFS].addr = fatCache.data;
    MPFSStubs[hMPFS].bytesRem = fatCache.len;

    return hMPFS;
}

/*****************************************************************************
  Function:
    void MPFSClose(MPFS_HANDLE hMPFS)

  Summary:
    Closes a file.

  Description:
    Closes a file and releases its stub back to the pool of available
    handles.

  Precondition:
    None

  Parameters:
    hMPFS - the file handle to be closed

  Returns:
    None
  ***************************************************************************/
void MPFSClose(MPFS_HANDLE hMPFS)
{
    if(hMPFS != 0u && hMPFS <= MAX_MPFS_HANDLES)
        MPFSStubs[hMPFS].addr = MPFS_INVALID;
}


/****************************************************************************
  Section:
    Data Reading Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    bool MPFSGet(MPFS_HANDLE hMPFS, uint8_t* c)

  Description:
    Reads a byte from a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read
    c - Where to store the byte that was read

  Return Values:
    true - The byte was successfully read
    false - No byte was read because either the handle was invalid or
            the end of the file has been reached.
  ***************************************************************************/
bool MPFSGet(MPFS_HANDLE hMPFS, uint8_t* c)
{
    // Make sure we're reading a valid address
    if(hMPFS > MAX_MPFS_HANDLES)
        return false;
    if( MPFSStubs[hMPFS].addr == MPFS_INVALID ||
        MPFSStubs[hMPFS].bytesRem == 0u)
        return false;

    if(c == NULL)
    {
        MPFSStubs[hMPFS].addr++;
        MPFSStubs[hMPFS].bytesRem--;
        return true;
    }


    // Read function for EEPROM
    #if defined(MPFS_USE_EEPROM)
        // For performance, cache the last read address
        if(MPFSStubs[hMPFS].addr != lastRead+1)
            XEEBeginRead(MPFSStubs[hMPFS].addr + MPFS_HEAD);
        *c = XEERead();
        lastRead = MPFSStubs[hMPFS].addr;
        MPFSStubs[hMPFS].addr++;
    #elif defined(MPFS_USE_SPI_FLASH)
        SPIFlashReadArray(MPFSStubs[hMPFS].addr + MPFS_HEAD, c, 1);
        MPFSStubs[hMPFS].addr++;
    #else
        #if defined(__XC16)
        {
            uint32_t addr;
            TCPIP_UINT32_VAL read;
            uint8_t i;

            // MPFS Images are addressed by the byte; Program memory by the word.
            //
            // Flash program memory is 24 bits wide and only even words are
            // implemented.  The upper byte of the upper word is read as 0x00.
            // Address in program memory of any given byte is (MPFSAddr * 2) / 3
            //
            // We will read 24 bits at a time, but need to support using only
            // fractions of the first and last byte.

            // Find the beginning address in program memory.
            addr = (MPFSStubs[hMPFS].addr / 3) << 1;

            // Find where to start in that first 3 bytes
            read.Val = (addr * 3) >> 1;
            if(read.Val == MPFSStubs[hMPFS].addr)
                i = 0;
            else if(read.Val+1 == MPFSStubs[hMPFS].addr)
                i = 1;
            else
                i = 2;

            // Add in the MPFS starting address offset
            addr += MPFS_HEAD;

            // Update the MPFS Handle
            MPFSStubs[hMPFS].addr++;

            // Read the uint32_t
            read.Val = ReadProgramMemory(addr & 0x00FFFFFF);
            *c = read.v[i];

        }
        #else
        {
            uint32_t dwHITECHWorkaround = MPFS_HEAD;
        *c = *((ROM uint8_t*)(MPFSStubs[hMPFS].addr+dwHITECHWorkaround));
            MPFSStubs[hMPFS].addr++;
        }
        #endif
    #endif

    MPFSStubs[hMPFS].bytesRem--;
    return true;
}

/*****************************************************************************
  Function:
    uint16_t MPFSGetArray(MPFS_HANDLE hMPFS, uint8_t* cData, uint16_t wLen)

  Description:
    Reads a series of bytes from a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read
    cData - where to store the bytes that were read
    wLen - how many bytes to read

  Returns:
    The number of bytes successfully read.  If this is less than wLen,
    an EOF occurred while attempting to read.
  ***************************************************************************/
uint16_t MPFSGetArray(MPFS_HANDLE hMPFS, uint8_t* cData, uint16_t wLen)
{
    // Make sure we're reading a valid address
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0;

    // Determine how many we can actually read
    if(wLen > MPFSStubs[hMPFS].bytesRem)
        wLen = MPFSStubs[hMPFS].bytesRem;

    // Make sure we're reading a valid address
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID || wLen == 0u)
        return 0;

    if(cData == NULL)
    {
        MPFSStubs[hMPFS].addr += wLen;
        MPFSStubs[hMPFS].bytesRem -= wLen;
        return wLen;
    }

    // Read the data
    #if defined(MPFS_USE_EEPROM)
        XEEReadArray(MPFSStubs[hMPFS].addr+MPFS_HEAD, cData, wLen);
        MPFSStubs[hMPFS].addr += wLen;
        MPFSStubs[hMPFS].bytesRem -= wLen;
        lastRead = MPFS_INVALID;
    #elif defined(MPFS_USE_SPI_FLASH)
        SPIFlashReadArray(MPFSStubs[hMPFS].addr+MPFS_HEAD, cData, wLen);
        MPFSStubs[hMPFS].addr += wLen;
        MPFSStubs[hMPFS].bytesRem -= wLen;
    #else
        #if defined(__XC16)
        {
            uint32_t addr;
            TCPIP_UINT32_VAL read;
            uint16_t count;
            uint8_t i;

            // MPFS Images are addressed by the byte; Program memory by the word.
            //
            // Flash program memory is 24 bits wide and only even words are
            // implemented.  The upper byte of the upper word is read as 0x00.
            // Address in program memory of any given byte is (MPFSAddr * 2) / 3
            //
            // We will read 24 bits at a time, but need to support using only
            // fractions of the first and last byte.

            // Find the beginning address in program memory.
            addr = (MPFSStubs[hMPFS].addr / 3) << 1;

            // Find where to start in that first 3 bytes
            read.Val = (addr * 3) >> 1;
            if(read.Val == MPFSStubs[hMPFS].addr)
                i = 0;
            else if(read.Val+1 == MPFSStubs[hMPFS].addr)
                i = 1;
            else
                i = 2;

            // Add in the MPFS starting address offset
            addr += MPFS_HEAD;

            // Update the MPFS Handle
            MPFSStubs[hMPFS].addr += wLen;
            MPFSStubs[hMPFS].bytesRem -= wLen;

            // Read the first uint32_t
            read.Val = ReadProgramMemory(addr & 0x00FFFFFF);
            addr += 2;

            // Copy values as needed
            for(count = wLen; count > 0; cData++, count--)
            {
                // Copy the next value in
                *cData = read.v[i++];

                // Check if a new uint32_t is needed
                if(i == 3 && count != 1)
                {// Read in a new uint32_t
                    read.Val = ReadProgramMemory(addr & 0x00FFFFFF);
                    addr += 2;
                    i = 0;
                }
            }

        }
        #else
        {
            uint32_t dwHITECHWorkaround = MPFS_HEAD;
            memcpypgm2ram(cData, (ROM void*)(MPFSStubs[hMPFS].addr + dwHITECHWorkaround), wLen);
            MPFSStubs[hMPFS].addr += wLen;
            MPFSStubs[hMPFS].bytesRem -= wLen;
        }
        #endif
    #endif

    return wLen;
}

/*****************************************************************************
  Function:
    bool MPFSGetLong(MPFS_HANDLE hMPFS, uint32_t* ul)

  Description:
    Reads a uint32_t or Long value from the MPFS.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read
    ul - where to store the uint32_t or long value that was read

  Returns:
    true - The byte was successfully read
    false - No byte was read because either the handle was invalid or
            the end of the file has been reached.
  ***************************************************************************/
bool MPFSGetLong(MPFS_HANDLE hMPFS, uint32_t* ul)
{
    return ( MPFSGetArray(hMPFS, (uint8_t*)ul, 4) == 4u );
}

/*****************************************************************************
  Function:
    bool MPFSSeek(MPFS_HANDLE hMPFS, uint32_t dwOffset, MPFS_SEEK_MODE tMode)

  Description:
    Moves the current read pointer to a new location.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle to seek with
    dwOffset - offset from the specified position in the specified direction
    tMode - one of the MPFS_SEEK_MODE constants

  Returns:
    true - the seek was successful
    false - either the new location or the handle itself was invalid
  ***************************************************************************/
bool MPFSSeek(MPFS_HANDLE hMPFS, uint32_t dwOffset, MPFS_SEEK_MODE tMode)
{
    uint32_t temp;

    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return false;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return false;

    switch(tMode)
    {
        // Seek offset bytes from start
        case MPFS_SEEK_START:
            temp = MPFSGetSize(hMPFS);
            if(dwOffset > temp)
                return false;

            MPFSStubs[hMPFS].addr = MPFSGetStartAddr(hMPFS) + dwOffset;
            MPFSStubs[hMPFS].bytesRem = temp - dwOffset;
            return true;

        // Seek forwards offset bytes
        case MPFS_SEEK_FORWARD:
            if(dwOffset > MPFSStubs[hMPFS].bytesRem)
                return false;

            MPFSStubs[hMPFS].addr += dwOffset;
            MPFSStubs[hMPFS].bytesRem -= dwOffset;
            return true;

        // Seek backwards offset bytes
        case MPFS_SEEK_REWIND:
            temp = MPFSGetStartAddr(hMPFS);
            if(MPFSStubs[hMPFS].addr < temp + dwOffset)
                return false;

            MPFSStubs[hMPFS].addr -= dwOffset;
            MPFSStubs[hMPFS].bytesRem += dwOffset;
            return true;

        // Seek so that offset bytes remain in file
        case MPFS_SEEK_END:
            temp = MPFSGetSize(hMPFS);
            if(dwOffset > temp)
                return false;

            MPFSStubs[hMPFS].addr = MPFSGetEndAddr(hMPFS) - dwOffset;
            MPFSStubs[hMPFS].bytesRem = dwOffset;
            return true;

        default:
            return false;
    }
}


/****************************************************************************
  Section:
    Data Writing Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    MPFS_HANDLE MPFSFormat(void)

  Summary:
    Prepares the MPFS image for writing.

  Description:
    Prepares the MPFS image for writing and locks the image so that other
    processes may not access it.

  Precondition:
    None

  Parameters:
    None

  Returns:
    An MPFS handle that can be used for MPFSPut commands, or
    MPFS_INVALID_HANDLE when the EEPROM failed to initialize for writing.

  Remarks:
    In order to prevent misreads, the MPFS will be inaccessible until
    MPFSClose is called.  This function is not available when the MPFS
    is stored in internal Flash program memory.
  ***************************************************************************/
#if defined(MPFS_USE_EEPROM) || defined(MPFS_USE_SPI_FLASH)
MPFS_HANDLE MPFSFormat(void)
{

    uint8_t i;

    // Close all files
    for(i = 0; i < MAX_MPFS_HANDLES; i++)
        MPFSStubs[i].addr = MPFS_INVALID;

    // Lock the image
    isMPFSLocked = true;

    #if defined(MPFS_USE_EEPROM)
        // Set FAT ptr for writing
        MPFSStubs[0].addr = 0;
        MPFSStubs[0].fatID = 0xffff;
        MPFSStubs[0].bytesRem = MPFS_WRITE_PAGE_SIZE - ( ((uint8_t)MPFSStubs[0].addr+MPFS_HEAD) & (MPFS_WRITE_PAGE_SIZE-1) );

        // Set up EEPROM for writing
        if( XEEBeginWrite(MPFSStubs[0].addr+MPFS_HEAD) == XEE_SUCCESS )
            return 0x00;

        return MPFS_INVALID_HANDLE;
    #else
        // Set up SPI Flash for writing
        SPIFlashBeginWrite(MPFS_HEAD);
        return 0x00;
    #endif
}
#endif

/*****************************************************************************
  Function:
    uint16_t MPFSPutArray(MPFS_HANDLE hMPFS, uint8_t *cData, uint16_t wLen)

  Description:
    Writes an array of data to the MPFS image.

  Precondition:
    MPFSFormat was sucessfully called.

  Parameters:
    hMPFS - the file handle for writing
    cData - the array of bytes to write
    wLen - how many bytes to write

  Returns:
    The number of bytes successfully written.

  Remarks:
    For EEPROM, the actual write may not initialize until the internal write
    page is full.  To ensure that previously written data gets stored,
    MPFSPutEnd must be called after the last call to MPFSPutArray.
  ***************************************************************************/
#if defined(MPFS_USE_EEPROM) || defined(MPFS_USE_SPI_FLASH)
uint16_t MPFSPutArray(MPFS_HANDLE hMPFS, uint8_t* cData, uint16_t wLen)
{
    #if defined(MPFS_USE_EEPROM)
        // Write to the EEPROM
        uint16_t count;

        for(count = 0; count < wLen; count++)
        {
            XEEWrite(cData[count]);

            MPFSStubs[hMPFS].addr++;
            MPFSStubs[hMPFS].bytesRem--;

            if(MPFSStubs[hMPFS].bytesRem == 0u)
            {
                MPFSPutEnd(false);
                isMPFSLocked = true;
                XEEBeginWrite(MPFSStubs[hMPFS].addr+MPFS_HEAD);
                MPFSStubs[hMPFS].bytesRem = MPFS_WRITE_PAGE_SIZE;
            }
        }

        return count;

    #else
        // Write to the SPI Flash
        SPIFlashWriteArray(cData, wLen);
        return wLen;
    #endif
}
#endif

/*****************************************************************************
  Function:
    void MPFSPutEnd(void)

  Description:
    Finalizes an MPFS writing operation.

  Precondition:
    MPFSFormat and MPFSPutArray were sucessfully called.

  Parameters:
    final - true if the application is done writing, false if MPFS2 called
        this function locally.

  Returns:
    None
  ***************************************************************************/
#if defined(MPFS_USE_EEPROM) || defined(MPFS_USE_SPI_FLASH)
void MPFSPutEnd(bool final)
{
    isMPFSLocked = false;

    #if defined(MPFS_USE_EEPROM)
        XEEEndWrite();
        while(XEEIsBusy());
    #endif

    if(final)
        _Validate();
}
#endif


/****************************************************************************
  Section:
    Meta Data Accessors
  ***************************************************************************/

/*****************************************************************************
  Function:
    static void _LoadFATRecord(uint16_t fatID)

  Description:
    Loads the FAT record for a specified handle.

  Precondition:
    None

  Parameters:
    fatID - the ID of the file whose FAT is to be loaded

  Returns:
    None

  Remarks:
    The FAT record will be stored in fatCache.
  ***************************************************************************/
static void _LoadFATRecord(uint16_t fatID)
{
    if(fatID == fatCacheID || fatID >= numFiles)
        return;

    // Read the FAT record to the cache
    MPFSStubs[0].bytesRem = 22;
    MPFSStubs[0].addr = 8 + numFiles*2 + fatID*22;
    MPFSGetArray(0, (uint8_t*)&fatCache, 22);
    fatCacheID = fatID;
}

/*****************************************************************************
  Function:
    uint32_t MPFSGetTimestamp(MPFS_HANDLE hMPFS)

  Description:
    Reads the timestamp for the specified file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The timestamp that was read as a uint32_t
  ***************************************************************************/
uint32_t MPFSGetTimestamp(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0x00000000;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return 0x00000000;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.timestamp;
}

/*****************************************************************************
  Function:
    uint32_t MPFSGetMicrotime(MPFS_HANDLE hMPFS)

  Description:
    Reads the microtime portion of a file's timestamp.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The microtime that was read as a uint32_t
  ***************************************************************************/
uint32_t MPFSGetMicrotime(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0x00000000;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return 0x00000000;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.microtime;
}

/*****************************************************************************
  Function:
    uint16_t MPFSGetFlags(MPFS_HANDLE hMPFS)

  Description:
    Reads a file's flags.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The flags that were associated with the file
  ***************************************************************************/
uint16_t MPFSGetFlags(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0x0000;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return 0x0000;

    //move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.flags;
}

/*****************************************************************************
  Function:
    uint32_t MPFSGetSize(MPFS_HANDLE hMPFS)

  Description:
    Reads the size of a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The size that was read as a uint32_t
  ***************************************************************************/
uint32_t MPFSGetSize(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0x00000000;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return 0x00000000;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.len;
}

/*****************************************************************************
  Function:
    uint32_t MPFSGetBytesRem(MPFS_HANDLE hMPFS)

  Description:
    Determines how many bytes remain to be read.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The number of bytes remaining in the file as a uint32_t
  ***************************************************************************/
uint32_t MPFSGetBytesRem(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0x00000000;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return 0x00000000;

    return MPFSStubs[hMPFS].bytesRem;
}

/*****************************************************************************
  Function:
    MPFS_PTR MPFSGetStartAddr(MPFS_HANDLE hMPFS)

  Description:
    Reads the starting address of a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The starting address of the file in the MPFS image
  ***************************************************************************/
MPFS_PTR MPFSGetStartAddr(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return 0;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return MPFS_INVALID;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.data;
}

/*****************************************************************************
  Function:
    MPFS_PTR MPFSGetEndAddr(MPFS_HANDLE hMPFS)

  Description:
    Determines the ending address of a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The address just after the file ends (start address of next file)
  ***************************************************************************/
MPFS_PTR MPFSGetEndAddr(MPFS_HANDLE hMPFS)
{
    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return MPFS_INVALID;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return MPFS_INVALID;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    return fatCache.data + fatCache.len;
}

/*****************************************************************************
  Function:
    bool MPFSGetFilename(MPFS_HANDLE hMPFS, uint8_t* cName, uint16_t wLen)

  Description:
    Reads the file name of a file that is already open.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to determine the file name
    cName - where to store the name of the file
    wLen - the maximum length of data to store in cName

  Return Values:
    true - the file name was successfully located
    false - the file handle provided is not currently open
  ***************************************************************************/
bool MPFSGetFilename(MPFS_HANDLE hMPFS, uint8_t* cName, uint16_t wLen)
{
    uint32_t addr;

    // Make sure a valid file is open
    if(hMPFS > MAX_MPFS_HANDLES)
        return false;
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID)
        return false;

    // Move to the point for reading
    _LoadFATRecord(MPFSStubs[hMPFS].fatID);
    addr = fatCache.string;
    MPFSStubs[0].addr = addr;
    MPFSStubs[0].bytesRem = 255;

    // Read the value and return
    MPFSGetArray(0, cName, wLen);
    return true;
}

/*****************************************************************************
  Function:
    uint32_t MPFSGetPosition(MPFS_HANDLE hMPFS)

  Description:
    Determines the current position in the file

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle for which to determine position

  Returns:
    The position in the file as a uint32_t (or MPFS_PTR)

  Remarks:
    Calling MPFSSeek(hMPFS, pos, MPFS_SEEK_START) will return the pointer
    to this position at a later time.  (Where pos is the value returned by
    this function.)
  ***************************************************************************/
uint32_t MPFSGetPosition(MPFS_HANDLE hMPFS)
{
    return MPFSStubs[hMPFS].addr - MPFSGetStartAddr(hMPFS);
}

/*****************************************************************************
  Function:
    uint16_t MPFSGetID(MPFS_HANDLE hMPFS)

  Description:
    Determines the ID in the FAT for a file.

  Precondition:
    The file handle referenced by hMPFS is already open.

  Parameters:
    hMPFS - the file handle from which to read the metadata

  Returns:
    The ID in the FAT for this file

  Remarks:
    Use this function in association with MPFSOpenID to quickly access file
    without permanently reserving a file handle.
  ***************************************************************************/
uint16_t MPFSGetID(MPFS_HANDLE hMPFS)
{
    return MPFSStubs[hMPFS].fatID;
}


/****************************************************************************
  Section:
    Utility Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
    void _Validate(void)

  Summary:
    Validates the MPFS Image

  Description:
    Verifies that the MPFS image is valid, and reads the number of
    available files from the image header.  This function is called on
    boot, and again after any image is written.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None
  ***************************************************************************/
static void _Validate(void)
{
    // If this function causes an Address Error Exception on 16-bit
    // platforms with code stored in internal Flash, make sure your
    // compiler memory model settings are correct.
    //
    // In MPLAB, choose Project Menu > Build Options > Project.
    // Select the MPLAB C30 tab and change Cagetory to Memory Model.
    // Ensure that Large Code Model is selected, and that the remaining
    //   options are set to Default.

    // Validate the image and update numFiles
    MPFSStubs[0].addr = 0;
    MPFSStubs[0].bytesRem = 8;
    MPFSGetArray(0, (uint8_t*)&fatCache, 6);
    if(!memcmppgm2ram((void*)&fatCache, (ROM void*)"MPFS\x02\x01", 6))
        MPFSGetArray(0, (uint8_t*)&numFiles, 2);
    else
        numFiles = 0;
    fatCacheID = MPFS_INVALID_FAT;
}
#endif //#if defined(STACK_USE_MPFS2)
