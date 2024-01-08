
#include "microshell.h"
#include "board/shell_config.h"

#define USB_SHELL 1

//--------------------------------------------------------------------+
// Microshell
//--------------------------------------------------------------------+

MICROSHELL ms;

#if USB_SHELL
#include "tusb.h"

static void utx(char c)
{
    tud_cdc_write(&c, 1);
}

static char urx(void)
{
    char c;
    if (tud_cdc_available()) {
      tud_cdc_read(&c, 1);
      return c;
    }
    return -1;
}


static void action_hook(MSCORE_ACTION action)
{
  (void)action;
}

#endif

void shell_setup(void) {

  microshell_init(&ms, utx, urx, action_hook);
}

void shell_task(void) {

    char buf[MSCONF_MAX_INPUT_LENGTH];
    tud_cdc_write_str("MicroShell>");
    microshell_getline(&ms, buf, sizeof(buf));
    tud_cdc_write_str("Your input text is '");
    tud_cdc_write_str(buf);
    tud_cdc_write_str("'");
    tud_cdc_write_str("\r\n");
}