
#include "ts.h"
#include "coretime.h"

static uint32_t us_max[ TASK_MAX ] = { 0 };
static uint64_t us_sum[ TASK_MAX ] = { 0 };
static uint32_t exe_cnt = 0;

static uint32_t ts_start_value[ TASK_MAX ];

void ts_start( TASK_E task_item )
{
    ts_start_value[ task_item ] = CoreTime32usGet();
}

void ts_end( TASK_E task_item )
{
    uint32_t ts_diff;
    
    ts_diff = CoreTime32usGet() - ts_start_value[ task_item ];
    
    us_sum[ task_item ] += ts_diff;
            
    if( ts_diff > us_max[ task_item ] )
    {
        us_max[ task_item ] = ts_diff;
    }
}

void exe_cycle_inc( void )
{
    exe_cnt++;
}

static uint32_t ts_log_fifo[255];
static uint8_t  ts_log_idx = 0;

void ts_log(void)
{
    ts_log_fifo[ts_log_idx] = CoreTime32usGet();
    ts_log_idx++;
}