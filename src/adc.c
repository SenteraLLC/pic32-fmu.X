////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Analog to Digital Converter (ADC) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "adc.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// Input power supply in millivolts.
static uint16_t battMillivolts = 0;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void ADCTask( void )
{
    static uint32_t an5SampleSum = 0;
    static uint8_t  sampleCount  = 0;

    static enum {
        SM_ADC_SAMP_AN5,
        SM_ADC_READ_AN5,
        SM_ADC_COMPUTE
    } adcState = SM_ADC_SAMP_AN5;

    switch( adcState )
    {
        case SM_ADC_SAMP_AN5:
        {
            // Select Batter Voltage (AN5) as ADC input.
            AD1CHSbits.CH0NA = 0;       // Channel 0 negative input is VR-.
            AD1CHSbits.CH0SA = 5;       // Channel 0 positive input is AN5.

            // Start sampling and auto conversion.
            AD1CON1SET = _AD1CON1_SAMP_MASK;
            
            adcState++;
            
            break;
        }
        case SM_ADC_READ_AN5:
        {
            // If conversion is done, get ADC value.
            if( AD1CON1bits.DONE == 1 )
            {
                an5SampleSum += ADC1BUF0;
                        
                adcState++;
            }
            
            break;
        }
        case SM_ADC_COMPUTE:
        {
            sampleCount++;

            if( sampleCount >= 64 )
            {
                // Compute actual input voltage. ====================

                // Constant values:
                //
                //      0.083 gain divider (1k / (11k + 1k))
                //        connects Vcc to AN5.
                //      3.2895 volt output from uModule = Vref+
                //
                // Computed Values:
                //
                //      Vcc * 0.083 = Vsample
                //
                //      Vsample = (Vref+ / 1024) * AN5
                //
                //      So...
                //
                //      Vcc = Vsample / 0.083
                //          = ((3.2895 / 1024) * AN5) / 0.083
                //          = 0.038704 * AN5
                //
                battMillivolts = ( ( 38704 * ( an5SampleSum >> 6 ) ) + 500 ) / 1000;

                // Zero the sample count and sum.
                an5SampleSum = 0;
                sampleCount = 0;
            }

            // Restart state machine.
            adcState = SM_ADC_SAMP_AN5;
            
            break;
        }
    }
    
    return;
}

uint16_t ADCVbattGet( void )
{
    return battMillivolts;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************
