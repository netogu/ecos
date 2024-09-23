#include "rtos.h"
#include "shell.h"

void shell_cmd_top_exec_cb(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[]) {
    (void) self;

    const char *header = USH_SHELL_FONT_STYLE_BOLD
                             USH_SHELL_FONT_COLOR_BLUE
                            "Task            Runtime(us)     Percentage\r\n"
                            "------------------------------------------\r\n"
                            USH_SHELL_FONT_STYLE_RESET;
    
    int header_len = strlen(header);
    int buffer_len = 40 * uxTaskGetNumberOfTasks() + header_len;
    char *buffer = pvPortMalloc(buffer_len);
    strcpy(buffer, header);
    vTaskGetRunTimeStatistics(buffer + header_len, buffer_len);
    ush_print(self, buffer);
    vPortFree(buffer);
}