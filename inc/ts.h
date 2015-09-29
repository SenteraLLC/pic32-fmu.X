/* 
 * File:   ts.h
 * Author: Jon Watson
 *
 * Created on September 29, 2015, 7:43 AM
 */

#ifndef TS_H
#define	TS_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <xc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    // Executive software cycle functions.
    ADC_TASK,
    SPI_TASK,
    UART_TASK,
    FMUCOMM_TASK,
    VN100_TASK,
    OEMSTAR_TASK,
    EMC1412_TASK,
    SBUS_TASK,
    RC_TASK,
    STATUS_TASK,
    SNODE_TASK,
    STACK_TASK,
    STACK_APPLICATION,
    
    // ISRs
    I2C_T4_ISR,
    SPI_T5_ISR,     
    SPI2_ISR,
    UART1_ISR,
    UART2_ISR,   
    
    TASK_MAX,
            
} TASK_E;
    
void ts_start( TASK_E task_item );
void ts_end( TASK_E task_item );
void exe_cycle_inc( void );
void ts_log(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TS_H */

