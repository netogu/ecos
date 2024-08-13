#include "board/bsp.h"
#include "drivers/stm32g4/hrtim.h"
#include "microshell.h"
#include "tusb.h"

/* FreeRTOS includes. */
#include "task.h"

// non-blocking read interface
static int ush_read(struct ush_object *self, char *ch) {
  // should be implemented as a FIFO

  if (tud_cdc_available() > 0) {
    *ch = tud_cdc_read_char();
    return 1;
  }
  return 0;
}

// non-blocking write interface
static int ush_write(struct ush_object *self, char ch) {
  // should be implemented as a FIFO
  return (tud_cdc_write_char(ch) == 1);
}

// I/O interface descriptor
static const struct ush_io_interface ush_iface = {
    .read = ush_read,
    .write = ush_write,
};

// working buffers allocations (size could be customized)
#define BUF_IN_SIZE 128
#define BUF_OUT_SIZE 128
#define PATH_MAX_SIZE 128

static char ush_in_buf[BUF_IN_SIZE];
static char ush_out_buf[BUF_OUT_SIZE];

// microshell instance handler
static struct ush_object ush;

// microshell descriptor
static const struct ush_descriptor ush_desc = {
    .io = &ush_iface,                          // I/O interface pointer
    .input_buffer = ush_in_buf,                // working input buffer
    .input_buffer_size = sizeof(ush_in_buf),   // working input buffer size
    .output_buffer = ush_out_buf,              // working output buffer
    .output_buffer_size = sizeof(ush_out_buf), // working output buffer size
    .path_max_length = PATH_MAX_SIZE,          // path maximum length (stack)
    .hostname = "stm32",                       // hostname (in prompt)
};

// drive enable callback
static void drv_en_callback(struct ush_object *self,
                            struct ush_file_descriptor const *file, int argc,
                            char *argv[]) {
  // arguments count validation
  if (argc != 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // arguments validation
  if (strcmp(argv[1], "1") == 0) {
    // turn gate driver on
    gpio_pin_set(&io.drive_enable);
  } else if (strcmp(argv[1], "0") == 0) {
    // turn gate driver off
    gpio_pin_clear(&io.drive_enable);
  } else {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }
}

// drive enable callback
static void mpwr_en_callback(struct ush_object *self,
                             struct ush_file_descriptor const *file, int argc,
                             char *argv[]) {
  // arguments count validation
  if (argc != 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // arguments validation
  if (strcmp(argv[1], "1") == 0) {
    // turn gate driver on
    gpio_pin_set(&io.test_pin0);
  } else if (strcmp(argv[1], "0") == 0) {
    // turn VM efuse driver off
    gpio_pin_clear(&io.test_pin0);
  } else {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }
}

// ocp threshold callback
static void ocp_exec_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file, int argc,
                              char *argv[]) {
  // arguments count validation
  if (argc < 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  float threshold_pc = atoi(argv[1]) / 100.0;
  if (threshold_pc > 1.0 || threshold_pc < 0.0) {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // TIM20.3 at 50% duty cycle maps to the OCP threshold at 100%
  float duty = threshold_pc * 1 / 2.0;
  duty > 0.5 ? 0.5 : duty < 0.0 ? 0.0 : duty;
  uint32_t period = TIM20->ARR;
  duty = period * duty + 0.5;
  TIM20->CCR3 = (uint32_t)duty;
}

// dpt test callback
static void dpt_exec_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file, int argc,
                              char *argv[]) {

  // arguments count validation
  if (argc < 5) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // struct hrtim_pwm *pwm = &pwma;
  struct hrtim_pwm *pwm = NULL;

  char *channel = argv[2];
  if (strcmp(channel, "a") == 0) {
    ush_printf(self, "Channel : A\r\n");
    pwm = &pwma;
  } else if (strcmp(channel, "b") == 0) {
    ush_printf(self, "Channel : B\r\n");
    pwm = &pwmb;
  } else if (strcmp(channel, "c") == 0) {
    ush_printf(self, "Channel : C\r\n");
    pwm = &pwmc;
  }

  if (pwm == NULL) {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  char *mode = argv[1];
  if (strcmp(mode, "buck") == 0) {
    ush_printf(self, "Mode : Buck\r\n");
  } else if (strcmp(mode, "boost") == 0) {
    ush_printf(self, "Mode : Boost\r\n");
    hrtim_pwm_swap_output(pwm);
  } else {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  int ton = atoi(argv[3]);
  ush_printf(self, "ton: %d ns\r\n", ton);

  int toff = atoi(argv[4]);
  ush_printf(self, "toff: %d ns\r\n", toff);

  int n = atoi(argv[5]);
  if (n <= 1) {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    ush_printf(self, "error: n has to be larger than 1\r\n");
    return;
  }
  ush_printf(self, "n: %d\r\n", n);

  // hrtim_pwm_stop(&pwm1);
  ton += pwm->deadtime_ns;
  toff += pwm->deadtime_ns;

  ton < 0 ? ton = 0 : ton;
  toff < 0 ? toff = 0 : toff;

  hrtim_pwm_stop(pwm);
  hrtim_pwm_set_frequency(pwm, 1000000000 / (ton + toff));
  hrtim_pwm_set_duty(pwm, ton * 100 / (ton + toff));
  hrtim_pwm_set_n_cycle_run(pwm, n);
  hrtim_pwm_start(pwm);
}

// reboot cmd file execute callback
static void reboot_exec_callback(struct ush_object *self,
                                 struct ush_file_descriptor const *file,
                                 int argc, char *argv[]) {
  NVIC_SystemReset();
}

// info file get data callback
size_t info_get_data_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file,
                              uint8_t **data) {
  static const char *info = "Minimal STM32G4 Shell\r\n";

  // return pointer to data
  *data = (uint8_t *)info;
  // return data size
  return strlen(info);
}

// root directory files descriptor
static const struct ush_file_descriptor root_files[] = {{
    .name = "info.txt", // info.txt file name
    .description = NULL,
    .help = NULL,
    .exec = NULL,
    .get_data = info_get_data_callback,
}};

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
        .description = "run double-pulse test", // optional file description
        .help = "usage: dpt mode(buck,boost) channel(a,b,c) ton(ns) toff(ns) "
                "n(pulses)\r\n",   // optional help manual
        .exec = dpt_exec_callback, // optional execute callback
    },
    {
        .name = "drv_en",
        .description = "enable/disable gate driver",
        .help = "usage: drv_en 1|0\r\n",
        .exec = drv_en_callback,
    },
    {
        .name = "mpwr_en",
        .description = "enable/disable VM efuse driver",
        .help = "usage: mpwr_en 1|0\r\n",
        .exec = mpwr_en_callback,
    },
    {
        .name = "ocp",
        .description = "sets ocp threshold (%)",
        .help = "usage: ocp < 0 to 100 >",
        .exec = ocp_exec_callback,
    }};

// root directory handler
static struct ush_node_object root;
// cmd commands handler
static struct ush_node_object cmd;

void shell_init(void) {

  // initialize microshell instance
  ush_init(&ush, &ush_desc);

  // add custom commands
  ush_commands_add(&ush, &cmd, cmd_files,
                   sizeof(cmd_files) / sizeof(cmd_files[0]));

  // mount root directory (root must be first)
  ush_node_mount(&ush, "/", &root, root_files,
                 sizeof(root_files) / sizeof(root_files[0]));
}

void shell_update(void) {
  // service microshell instance
  ush_service(&ush);
}
