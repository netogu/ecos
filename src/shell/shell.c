#include "rtos.h"
#include "bsp.h"
#include "shell.h"
#include <inttypes.h> // for PRIu64

#define NOCHAR '\0'
#define SHELL_HOSTNAME "board"


#define CLI_UART
// #define CLI_USB



extern void shell_cmd_top_exec_cb(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[]);


// non-blocking read interface
static int ush_read(struct ush_object *self, char *ch)
{
    char readchar = NOCHAR;
    

    #ifdef CLI_USB
    if (tud_cdc_connected() && tud_cdc_available() > 0) {
        // Write single byte if mutex is available
        readchar = cli_usb_getc();
    }
    #else
    readchar = cli_uart_getc();
    #endif

    if (readchar != NOCHAR) {
        *ch = readchar;
        return 1;
    }
    return 0;
}

// non-blocking write interface
static int ush_write(struct ush_object *self, char ch)
{
    #ifdef CLI_USB
    return cli_usb_putc(ch);
    #else
    return cli_uart_putc(ch);
    #endif
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
    .hostname = SHELL_HOSTNAME,                          // hostname (in prompt)
};

// drive enable callback
static void drv_en_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    struct board_descriptor *brd = board_get_descriptor();

    // arguments count validation
    if (argc != 2) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    // arguments validation
    if (strcmp(argv[1], "1") == 0) {
        // turn gate driver on
        gpio_pin_set(&brd->io.drive_enable);
    } else if (strcmp(argv[1], "0") == 0) {
        // turn gate driver off
        gpio_pin_clear(&brd->io.drive_enable);
    } else {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }
}

// drive enable callback
static void mpwr_en_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    struct board_descriptor *brd = board_get_descriptor();

    // arguments count validation
    if (argc != 2) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    // arguments validation
    if (strcmp(argv[1], "1") == 0) {
        // turn gate driver on
        gpio_pin_set(&brd->io.test_pin0);
    } else if (strcmp(argv[1], "0") == 0) {
        // turn VM efuse driver off
        gpio_pin_clear(&brd->io.test_pin0);
    } else {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }
}

// ocp threshold callback
static void ocp_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    // arguments count validation
    if (argc < 2) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    float threshold_pc = atoi(argv[1])/100.0;
    if ( threshold_pc > 1.0 || threshold_pc < 0.0 ) {
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    // TIM20.3 at 50% duty cycle maps to the OCP threshold at 100%
    float duty = threshold_pc * 1/2.0;
    duty = (duty > 0.5) ? 0.5 : ((duty < 0.0) ? 0.0 : duty);

    uint32_t period = TIM20->ARR;
    duty = period * duty + 0.5;
    TIM20->CCR3 = (uint32_t) duty;

}

// dpt test callback
static void dpt_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    struct board_descriptor *brd = board_get_descriptor();

    // arguments count validation
    if (argc < 3) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

    pwm_t *pwm = &brd->pwma;

    int ton = atoi(argv[1]);
    ush_printf(self, "ton: %d ns\r\n", ton);
    int toff = atoi(argv[2]);
    ush_printf(self, "toff: %d ns\r\n", toff);
    int n = atoi(argv[3]);
    if (n <= 1) {
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        ush_printf(self, "error: n has to be larger than 1\r\n");
        return;
    }
    ush_printf(self, "n: %d\r\n", n);


    // hrtim_pwm_stop(&pwm1);
    // get deadtime from HRTIM registers and convert to ns
    // uint32_t deadtime_ns = HRTIM1->sCommonRegs.DTR & HRTIM_DTR_DTF_Msk;
    uint32_t deadtime_ns = 0;

    ton += deadtime_ns;
    toff += deadtime_ns;

    ton < 0 ? ton = 0 : ton;
    toff < 0 ? toff = 0 : toff;

    pwm_set_frequency(pwm, 1000000000/(ton + toff));
    pwm_set_duty(pwm, ton*100/(ton + toff));
    pwm_set_n_cycle_run(pwm, n);
    pwm_start(pwm);

}

// PWMA Set Duty Callback
static void pwma_set_duty_cb(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    struct board_descriptor *brd = board_get_descriptor();

    // arguments count validation
    if (argc < 2) {
        // return predefined error message
        ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
        return;
    }

     pwm_t *pwm = &brd->pwma;

    int duty = atoi(argv[1]);
    ush_printf(self, "duty: %d %%\r\n", duty);

    pwm_set_duty(pwm, (float)duty/100.0);

}


// reboot cmd file execute callback
static void reboot_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    NVIC_SystemReset();
}

// clear cmd file execute callback
static void clear_exec_callback(struct ush_object *self, struct ush_file_descriptor const *file, int argc, char *argv[])
{
    ush_print(self,"\033[2J\033[1;1H"); 
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

// ADC_DR file get data callback
size_t ADC_DR_get_data_callback(struct ush_object *self, struct ush_file_descriptor const *file, uint8_t **data)
{
    // get board descriptor
    struct board_descriptor *brd = board_get_descriptor();

    char dr[16];
    // uint16_t value = *brd->ain.vbus.data;
    uint16_t value = 2134;
    ush_printf(self, "ADC1: %d \r\n", value);


    // return pointer to data
    *data = (uint8_t*)dr;
    // return data size
    return strlen(dr);
}

// root directory files descriptor
static const struct ush_file_descriptor root_files[] = {
    {
        .name = "info.txt",                     // info.txt file name
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = info_get_data_callback,
    },
    {
        .name = "ADC_DR",                     // info.txt file name
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = ADC_DR_get_data_callback,
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
        .name = "clear",
        .description = "clear terminal",
        .help = NULL,
        .exec = clear_exec_callback,
    },
    {
        .name = "top",
        .description = "Task Monitor",
        .help = NULL,
        .exec = shell_cmd_top_exec_cb,
    },
    {
        .name = "dpt",                       
        .description = "run double-pulse test",            // optional file description
        .help = "usage: dpt channel(a,b,c) ton(ns) toff(ns) n(pulses)\r\n",            // optional help manual
        .exec = dpt_exec_callback,           // optional execute callback
    },
    {   .name = "drv_en",
        .description = "enable/disable gate driver",
        .help = "usage: drv_en 1|0\r\n",
        .exec = drv_en_callback,
    },
    {   .name = "mpwr_en",
        .description = "enable/disable VM efuse driver",
        .help = "usage: mpwr_en 1|0\r\n",
        .exec = mpwr_en_callback,
    },
    {
        .name = "ocp",
        .description = "sets ocp threshold (%)",
        .help = "usage: ocp < 0 to 100 >",
        .exec = ocp_exec_callback,
    },
    {
        .name = "pwma_duty",
        .description = "set pwma duty cycle (%)",
        .help = "usage: pwma_duty <duty:0-100>",
        .exec = pwma_set_duty_cb,
    }
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


void inline shell_update(void)
{
    // service microshell instance
    ush_service(&ush);
}


char * timestamp(void)
{
    static char timestamp_msg[16];
    uint64_t usec =  timer_us_get();
    snprintf(timestamp_msg, sizeof(timestamp_msg), "[%11u\t] ", (uint32_t)usec);
    return timestamp_msg;
}

