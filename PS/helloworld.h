#ifndef HELLOWORLD_H
#define HELLOWORLD_H

#include "xstatus.h"
#include "xrfdc.h"

extern XRFdc InstancePtr_RFSOC;

int metal_log_init(void);
int rf_Init(void);

#endif
