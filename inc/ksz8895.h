/*******************************************************************************
/
/   Filename:   ksz8895.h
/
*******************************************************************************/

#ifndef KSZ8895_H
#define KSZ8895_H

#include <inttypes.h>


// Function Prototypes =========================================================

int KSZ8895Init();
int KSZ8895ReadReg(uint8_t address, uint8_t *data);
int KSZ8895WriteReg(uint8_t address, uint8_t data);
void KSZ8895Reset();


#endif  /* KSZ8895_H */

