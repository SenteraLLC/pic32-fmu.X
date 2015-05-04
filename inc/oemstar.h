/*******************************************************************************
/
/   Filename:   oemstar.h
/
*******************************************************************************/

#ifndef OEMSTAR_H
#define	OEMSTAR_H

#include "stdtypes.h"


//==============================================================================


typedef enum {
    LOGLIST         = 5,    // List of system logs
    GPSEPHEM        = 7,    // GPS decoded ephemeris data
    IONUTC          = 8,    // Ionospheric and UTC data
    RTCA1           = 10,   // RTCA, Type 1 Differential GPS Corrections
    CLOCKMODEL      = 16,   // Current clock model status
    RAWGPSSUBFRAME  = 25,   // Raw subframe data
    CLOCKSTEERING   = 26,   // Clock steering status
    VERSION         = 37,   // Receiver hardware and software version numbers
    RAWEPHEM        = 41,   // Raw ephemeris
    BESTPOS         = 42,   // Best position data
    RANGE           = 43,   // Satellite range information
    PSRPOS          = 47,   // Pseudorange position
    SATVIS          = 48,   // Satellite visibility
    PORTSTATS       = 72,   // COM or USB port statistics
    ALMANAC         = 73,   // Current decoded almanac
    RAWALM          = 74,   // Raw almanac data
    TRACKSTAT       = 83,   // Satellite tracking status
    RXSTATUS        = 93,   // Receiver status
    RXSTATUSEVENT   = 94,   // Status event indicator
    BESTVEL         = 99,   // Best available velocity data
    PSRVEL          = 100,  // Pseudorange velocity
    TIME            = 101,  // Receiver time data
    RTCM1           = 107,  // RTCM, Type 1 Differential GPS Corrections
    RXCONFIG        = 128,  // Receiver configuration
    RTCM16          = 129,  // RTCM, Type 16 Special Message
    RANGECMP        = 140,  // Compressed version of the RANGE log
    NAVIGATE        = 161,  // User navigation data
    AVEPOS          = 172,  // Position averaging
    PSRDOP          = 174,  // Pseudorange DOP
    MARKPOS         = 181,  // Position at time of mark input event
    VALIDMODELS     = 206,  // Valid model information
    GPALM           = 217,  // NMEA, Almanac Data
    GPGGA           = 218,  // NMEA, GPS Fix Data and Undulation
    GPGLL           = 219,  // NMEA, Geographic Position
    GPGRSNMEA       = 220,  // GPS Range Residuals for Each Satellite
    GPGSA           = 221,  // NMEA, GPS DOP and Active Satellites
    GPGST           = 222,  // NMEA, Pseudorange Measurement Noise Statistics
    GPGSV           = 223,  // NMEA, GPS Satellites in View
    GPRMB           = 224,  // NMEA, Navigation Information
    GPRMC           = 225,  // NMEA, GPS Specific Information
    GPVTG           = 226,  // NMEA, Track Made Good and Ground Speed
    GPZDA           = 227,  // NMEA, UTC Time and Date
    MARKTIME        = 231,  // Time of mark input event
    PASSCOM1        = 233,  // Pass-through log
    PASSCOM2        = 234,  // Pass-through log
    BESTXYZ         = 241,  // Best available Cartesian position and velocity
    PSRXYZ          = 243,  // Pseudorange Cartesian position and velocity
    SATXYZ          = 270,  // SV position in ECEF Cartesian coordinates
    RTCM9           = 275,  // RTCM, Type 9 Partial Diff GPS Corrections
    SBAS0           = 290,  // Remove PRN from the solution
    SBAS1           = 291,  // PRN mask assignments
    SBAS10          = 292,  // Degradation factor
    SBAS12          = 293,  // SBAS network time and UTC
    SBAS17          = 294,  // GEO almanac message
    SBAS18          = 295,  // IGP mask
    SBAS2           = 296,  // Fast correction slots 0-12
    SBAS24          = 297,  // Mixed fast/slow corrections
    SBAS25          = 298,  // Long-term slow satellite corrections
    SBAS26          = 299,  // Ionospheric delay corrections
    SBAS27          = 300,  // SBAS service message
    SBAS3           = 301,  // Fast correction slots 13-25
    SBAS4           = 302,  // Fast correction slots 26-38
    SBAS5           = 303,  // Fast correction slots 39-50
    SBAS6           = 304,  // Integrity message
    SBAS7           = 305,  // Fast correction degradation
    SBAS9           = 306,  // GEO navigation message
    SBASCORR        = 313,  // SBAS range corrections used
    COMCONFIG       = 317,  // Current COM port configuration
    RTCAEPHEM       = 347,  // RTCA, Type 7 Ephemeris and Time Information
    RTCADATA1       = 392,  // Type 1 Differential GPS Corrections
    RTCADATAEPHEM   = 393,  // Type 7 Ephemeris and Time Information
    RTCMDATA1       = 396,  // Type 1 Differential GPS Corrections
    RTCMDATA16      = 398,  // Type 16 Special Message
    RTCMDATA9       = 404,  // Type 9 Partial Differential GPS Corrections
    PASSXCOM1       = 405,  // Pass-through log
    PASSXCOM2       = 406,  // Pass-through log
    RAWGPSWORD      = 407,  // Raw navigation word
    PDPPOS          = 469,  // PDP filter position
    PDPVEL          = 470,  // PDP filter velocity
    PDPXYZ          = 471,  // PDP filter Cartesian position and velocity
    GPGGALONG       = 521,  // NMEA, GPS Fix Data, Extra Precision & Undulation
    PASSUSB1        = 607,  // Pass-through logs (for receivers supporting USB)
    PASSUSB2        = 608,  // Pass-through logs (for receivers supporting USB)
    PASSUSB3        = 609,  // Pass-through logs (for receivers supporting USB)
    GLOALMANAC      = 718,  // GLONASS decoded almanac data
    GLOCLOCK        = 719,  // GLONASS clock information
    GLORAWALM       = 720,  // Raw GLONASS almanac data
    GLORAWFRAME     = 721,  // Raw GLONASS frame data
    GLORAWSTRING    = 722,  // Raw GLONASS string data
    GLOEPHEMERIS    = 723,  // GLONASS ephemeris data
    BESTUTM         = 726,  // Best available UTM data
    RTCM1005        = 765,  // Stationary RTK Base Station Ant Ref Point (ARP)
    RTCM1006        = 768,  // Stationary RTK Base Station ARP with Ant Height
    RTCM1004        = 770,  // Extended L1 and L2 GPS RTK Observables
    RTCM1001        = 772,  // L1-Only GPS RTK Observables
    RTCM1002        = 774,  // Extended L1-Only GPS RTK Observables
    RTCM1003        = 776,  // L1 And L2 GPS RTK Observables
    GLORAWEPHEM     = 792,  // Raw GLONASS ephemeris data
    PASSXCOM3       = 795,  // Pass through log
    RTCM1007        = 852,  // Ext Ant Descriptor and Setup Information
    RTCM1008        = 854,  // Ext Ant Ref Station Description & Serial Number
    GLMLA           = 859,  // NMEA, GLONASS Almanac Data
    RTCM31          = 864,  // RTCM, Type 31 Differential GLONASS Corrections
    RTCMDATA31      = 868,  // Type 31 GLONASS Differential Corrections
    RTCM36          = 875,  // RTCM, Type 36 Special Extended Message
    RTCM36T         = 877,  // RTCM, Type 36T Special Extended Text Message
    RTCMDATA36      = 879,  // Type 36 Special Message
    PSRTIME         = 881,  // Time offsets from the pseudorange filter
    RTCM1009        = 885,  // GLONASS L1-Only RTK
    RTCM1010        = 887,  // Extended GLONASS L1-Only RTK
    RTCM1011        = 889,  // GLONASS L1/L2 RTK
    RTCM1012        = 891,  // Extended GLONASS L1/L2 RTK
    RTCM1019        = 893,  // GPS Ephemerides
    RTCM1020        = 895,  // GLONASS EPHEMERIDES
    RTCM59GLO       = 903,  // RTCM, NovAtel proprietary GLONASS differential
    RTCMDATA59GLO   = 905,  // NovAtel proprietary GLONASS diff corrections
    RAWSBASFRAME    = 973,  // Raw SBAS frame data
    SATVIS2         = 1043, // Satellite visibility
    RTCM1033        = 1097, // Receiver and antenna descriptors
    CHANCONFIGLIST  = 1148, // All available channel configurations
    PSRSATS         = 1162, // Satellites used in PSRPOS solution
    PSRDOP2         = 1163, // Pseudorange DOP
    CLOCKMODEL2     = 1170, // Clock bias
    PDPSATS         = 1234, // Satellites used in PDPPOS solution
    SOFTLOADSTATUS  = 1235, // Status of the softload process
    RAIMSTATUS      = 1286, // RAIM status
    AUTHCODES       = 1348, // List of authorization codes
    RTCM1071        = 1472, // MSM1, GPS Code Measurements
    RTCM1072        = 1473, // MSM2, GPS Phase Measurements
    RTCM1073        = 1474, // MSM3, GPS Code and Phase Measurements
    RTCM1074        = 1475, // MSM4, GPS Code, Phase and CNR Measurements
    RTCM1075        = 1476, // MSM5, GPS Code, Phase, 
                            //   CNR & Doppler Measurements
    RTCM1076        = 1477, // MSM6, Ext GPS Code, Phase & CNR Measurements
    RTCM1077        = 1478, // MSM7, Ext GPS Code, Phase, 
                            //   CNR & Doppler Measurements
    RTCM1081        = 1479, // MSM1, GLONASS Code Measurements
    RTCM1082        = 1480, // MSM2, GLONASS Phase Measurements
    RTCM1083        = 1481, // MSM3, GLONASS Code and Phase Measurements
    RTCM1084        = 1482, // MSM4, GLONASS Code, Phase and CNR Measurements
    RTCM1085        = 1483, // MSM5, GLONASS Code, Phase, 
                            //   CNR and Doppler Measurements
    RTCM1086        = 1484, // MSM6, Extended GLONASS Code, 
                            //   Phase and CNR Measurements
    RTCM1087        = 1485, // MSM7, Extended GLONASS Code, Phase, 
                            //   CNR and Doppler Measurements
} OEMSTAR_MSG_ID_E;


//==============================================================================


typedef struct {
    uint8_t     sync0;      // Always 0xAA.
    uint8_t     sync1;      // Always 0x44.
    uint8_t     sync2;      // Always 0x12
    uint8_t     headerLen;  // Length of the header.
    uint16_t    msgID;      // This is the message ID number of the log.
} OEMSTAR_MSG_HEADER;


//==============================================================================


// Best Available Cartesian Position and Velocity

typedef struct {
    // header
    // enum solution status
    // enum position type
    double      posX;       // Position X-coordinate (m)
    double      posY;       // Position Y-coordinate (m)
    double      posZ;       // Position Z-coordinate (m)
    float       stdPX;      // Standard deviation of P-X (m)
    float       stdPY;      // Standard deviation of P-Y (m)
    float       stdPZ;      // Standard deviation of P-Z (m)
    // vsol status
    // vel type
    double      velX;       // Velocity vector along X-axis (m/s)
    double      velY;       // Velocity vector along Y-axis (m/s)
    double      velZ;       // Velocity vector along Z-axis (m/s)
    float       stdVX;      // Standard deviation of V-X (m/s)
    float       stdVY;      // Standard deviation of V-Y (m/s)
    float       stdVZ;      // Standard deviation of V-Z (m/s)
    char        stnID[4];   // Base station identification
    float       vLatency;   // A measure of the latency in the velocity time
                            //   tag in seconds. It should be subtracted from
                            //   the time to give improved results.
    float       diffAge;    // Differential age in seconds.
    float       solAge;     // Solution age in seconds.
    uint8_t     numSVs;     // # of satellite vehicles tracked.
    uint8_t     numSolnSVs; // # of satellite vehicles used in solution.
    uint8_t     numGGL1;    // # of GPS L1 plus GLONASS L1 used in solution.
    uint8_t     reserved0;  // Reserved.
    uint8_t     reserved1;  // Reserved.
    uint8_t     extSolStat; // Extended solution status.
    uint8_t     reserved2;  // Reserved.
    uint8_t     sigMask;    // Signals used mask - if 0, signals used
                            //   in solution are unknown.
    uint32_t    ncrc;       // 32-bit CRC
} OEMSTAR_BESTXYZ;


// Function Prototypes =========================================================

void OEMStarTask();


#endif	/* OEMSTAR_H */

