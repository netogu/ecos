#include "microshell.h"
#include "board/bsp.h"
#include "tusb.h"

/* FreeRTOS includes. */
#include "task.h"

// non-blocking read interface
static int ush_read(struct ush_object *self, char *ch)
{
    // should be implemented as a FIFO

    if (tud_cdc_available() > 0) {
        *ch = tud_cdc_read_char();
        return 1;
    }
    return 0;
}

// non-blocking write interface
static int ush_write(struct ush_object *self, char ch)
{
    // should be implemented as a FIFO
    return (tud_cdc_write_char(ch) == 1);
}

// I/O interface descriptor
static const struct ush_io_interface ush_iface = {
    .read = ush_read,
    .write = ush_write,
};

// working buffers allocations (size could be customized)
    #define BUF_IN_SIZE    64
    #define BUF_OUT_SIZE   64
    #define PATH_MAX_SIZE  64

static char ush_in_buf[BUF_IN_SIZE];
static char ush_out_buf[BUF_OUT_SIZE];

// microshell instance handler
static struct ush_object ush;

// microshell descriptor
static const struct ush_descriptor ush_desc = {
    .io = &ush_iface,                           // I/O interface pointer
    .input_buffer = ush_in_buf,                 // working input buffer
    .input_buffer_size = sizeof(ush_in_buf),    // working input buffer size
    .output_buffer = ush_out_buf,               // working output buffer
    .output_buffer_size = sizeof(ush_out_buf),  // working output buffer size
    .path_max_length = PATH_MAX_SIZE,           // path maximum length (stack)
    .hostname = "stm32",                      // hostname (in prompt)
};

// toggle file execute callback
static void dpt_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{

    // arguments count validation
    if (argc < 3) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }


    int ton = atoi(argv[1]);
    ush_printf(self, "ton: %d ns\r\n", ton);
    int toff = atoi(argv[2]);
    ush_printf(self, "toff: %d ns\r\n", toff);
    int n = atoi(argv[3]);
    ush_printf(self, "n: %d\r\n", n);

    // hrtim_pwm_stop(&pwm1);
    ton += pwm1.deadtime_ns;
    toff += pwm1.deadtime_ns;

    ton < 0 ? ton = 0 : ton;
    toff < 0 ? toff = 0 : toff;
    if (toff < ton) {
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }
    hrtim_pwm_set_frequency(&pwm1, 1000000000/(ton + toff));
    hrtim_pwm_set_duty(&pwm1, ton*100/(ton + toff));
    hrtim_pwm_set_n_cycle_run(&pwm1, n);
    hrtim_pwm_start(&pwm1);

}

// reboot cmd file execute callback
static void reboot_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    ush_print(self, "error: reboot not supported...");
}


// info file get data callback
size_t info_get_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t **data)
{
    static const char *info = "Minimal STM32G4 Shell\r\n";

    // return pointer to data
    *data = (uint8_t*)info;
    // return data size
    return strlen(info);
}




// root directory files descriptor
static const struct ush_file_descriptor root_files[] = {
    {
        .name = "info.txt",                     // info.txt file name
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = info_get_data_callback,
    }
};

// cmd files descriptor
static const struct ush_file_descriptor cmd_files[] = {
    {
        .name = "reboot",
        .description = "reboot device",
        .help = NULL,
        .exec = reboot_exec_callback,
    },
    {
        .name = "dpt",                       
        .description = "run double-pulse test",            // optional file description
        .help = "usage: dpt ton(ns) toff(ns) n(pulses)\r\n",            // optional help manual
        .exec = dpt_exec_callback,           // optional execute callback
    },
};

// root directory handler
static struct ush_node_object root;
// cmd commands handler
static struct ush_node_object cmd;

void shell_init(void)
{

    // initialize microshell instance
    ush_init(&ush, &ush_desc);

    // add custom commands
    ush_commands_add(&ush, &cmd, cmd_files, sizeof(cmd_files) / sizeof(cmd_files[0]));

    // mount root directory (root must be first)
    ush_node_mount(&ush, "/", &root, root_files, sizeof(root_files) / sizeof(root_files[0]));

}

void shell_update(void)
{
    // service microshell instance
    ush_service(&ush);
}

