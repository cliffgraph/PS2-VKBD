#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>
#include "usb_device_cdc.h"

void APP_Initialize(void);
void APP_Tasks(void);

typedef enum
{
    APP_SYSTEM_STATE_USB_START,
    APP_SYSTEM_STATE_USB_SUSPEND,
    APP_SYSTEM_STATE_USB_RESUME
} APP_SYSTEM_STATE;

void APP_SYSTEM_Initialize( APP_SYSTEM_STATE state );

#endif
