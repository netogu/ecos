#include "hal.h"
#include "tusb.h"

#define USB_STR_SERIALNO_LEN    8
#define NOCHAR                  '\0'

static StaticSemaphore_t usb_mutex_buffer;
SemaphoreHandle_t usb_mutex;

static char usb_serialno[USB_STR_SERIALNO_LEN] = "00000000";

// void usb_get_serialno(void) {
//     uint8_t id[8];
//     HAL_GetUID(id);
//     snprintf(usb_serialno, USB_STR_SERIALNO_LEN, "%02X%02X%02X%02X%02X%02X%02X%02X",
//              id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
// }

void cli_usb_init(void) {
    // usb_get_serialno();
    usb_mutex = xSemaphoreCreateMutexStatic(&usb_mutex_buffer);
    if (usb_mutex == NULL) {
        while (1);
    }
    tusb_init();
}

int cli_usb_putc(char tx_char) {
	int status = 0;

	// check if connected
	if (tud_cdc_connected()) {
		// write single char (byte) if mutex is available
		if(xSemaphoreTake(usb_mutex, 10) == pdTRUE) {
			if (tud_cdc_write_char(tx_char) == 1) {
				tud_cdc_write_flush();
				status = 1;
			}
			xSemaphoreGive(usb_mutex);
		}
	}

	return status;
}

char cli_usb_getc(void) {
	char readchar = NOCHAR;

	// check if connected and there are bytes available
	if (tud_cdc_connected() && tud_cdc_available() > 0) {
		// read single char (byte) if mutex is available
		if(xSemaphoreTake(usb_mutex, 10) == pdTRUE) {
			readchar = tud_cdc_read_char();
			xSemaphoreGive(usb_mutex);
		}
	}

	return readchar;
}