#include "boot.h"	/* bootloader generic header	*/
#include "usb_lib.h"
#include "usb_pwr.h"

#if (BOOT_COM_USB_ENABLE == 0)

volatile uint32_t bDeviceState = UNCONNECTED; 
DEVICE_PROP Device_Property;
DEVICE Device_Table;
USER_STANDARD_REQUESTS User_Standard_Requests;
void UsbReceivePipeBulkOUT(void) { }
void UsbTransmitPipeBulkIN(void) { }

#endif
