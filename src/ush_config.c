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
    #define BUF_IN_SIZE    32
    #define BUF_OUT_SIZE   32
    #define PATH_MAX_SIZE  32

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
static void toggle_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    // simple toggle led, without any arguments validation
    gpio_pin_toggle(&gpios.led_green);

}

// reboot cmd file execute callback
static void reboot_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    ush_print(self, "error: reboot not supported...");
}

// set file execute callback
static void set_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    // arguments count validation
    if (argc != 2) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    // arguments validation
    if (strcmp(argv[1], "1") == 0) {
        // turn led on
        gpio_pin_set(&gpios.led_green);
    } else if (strcmp(argv[1], "0") == 0) {
        // turn led off
        gpio_pin_clear(&gpios.led_green);
    } else {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }
}

// info file get data callback
size_t info_get_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t **data)
{
    static const char *info = "Use MicroShell and make fun!\r\n";

    // return pointer to data
    *data = (uint8_t*)info;
    // return data size
    return strlen(info);
}

// led file get data callback
size_t led_get_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t **data)
{
    // read current led state
    bool state = gpio_pin_read(&gpios.led_green);
    // return pointer to data
    *data = (uint8_t*)((state) ? "1\r\n" : "0\r\n");
    // return data size

    return strlen((char*)(*data));
}

// led file set data callback
void led_set_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t *data, size_t size)
{
    // data size validation
    if (size < 1)
        return;

    // arguments validation
    if (data[0] == '1') {
        // turn led on
        gpio_pin_set(&gpios.led_green);
    } else if (data[0] == '0') {
        // turn led off
        gpio_pin_clear(&gpios.led_green);
    }
}

// time file get data callback
size_t time_get_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t **data)
{
    static char time_buf[16];
    // read current time
    // long current_time = xTaskGetTickCount();
    long current_time = 1000;
    // convert
    snprintf(time_buf, sizeof(time_buf), "%ld\r\n", current_time);
    time_buf[sizeof(time_buf) - 1] = 0;
    // return pointer to data
    *data = (uint8_t*)time_buf;
    // return data size
    return strlen((char*)(*data));
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

// bin directory files descriptor
static const struct ush_file_descriptor bin_files[] = {
    {
        .name = "toggle",                       // toogle file name
        .description = "toggle led",            // optional file description
        .help = "usage: toggle\r\n",            // optional help manual
        .exec = toggle_exec_callback,           // optional execute callback
    },
    {
        .name = "set",                          // set file name
        .description = "set led",
        .help = "usage: set {0,1}\r\n",
        .exec = set_exec_callback
    },
};

// dev directory files descriptor
static const struct ush_file_descriptor dev_files[] = {
    {
        .name = "led",
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = led_get_data_callback,      // optional data getter callback
        .set_data = led_set_data_callback,      // optional data setter callback
    },
    {
        .name = "time",
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = time_get_data_callback,
    },
};

// cmd files descriptor
static const struct ush_file_descriptor cmd_files[] = {
    {
        .name = "reboot",
        .description = "reboot device",
        .help = NULL,
        .exec = reboot_exec_callback,
    },
};

// root directory handler
static struct ush_node_object root;
// dev directory handler
static struct ush_node_object dev;
// bin directory handler
static struct ush_node_object bin;

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
    // mount dev directory
    ush_node_mount(&ush, "/dev", &dev, dev_files, sizeof(dev_files) / sizeof(dev_files[0]));
    // mount bin directory
    ush_node_mount(&ush, "/bin", &bin, bin_files, sizeof(bin_files) / sizeof(bin_files[0]));

}

void shell_update(void)
{
    // service microshell instance
    ush_service(&ush);
}


