#ifndef __USB_PROCESS_H
#define __USB_PROCESS_H

#include "global.h"
#include "main.h"

#include "stm32h7xx_it.h"
#include "usbd_cdc_if.h"


void Process_USB_Receive_Buffer();

#endif
