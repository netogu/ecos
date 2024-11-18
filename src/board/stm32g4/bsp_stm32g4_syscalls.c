
#include "bsp.h"

//------------------------------------------------------
// Syscalls
//------------------------------------------------------

// printf redirect to UART
void _putchar(char character) {

  board_t *brd = board_get_handle();

#ifdef SHELL_INTERFACE_USB
  tud_cdc_write_char(character);
#elif defined(SHELL_INTERFACE_USART3)
  uart_write(&brd.usart3, (uint8_t *)&character, 1);
#else
  uart_write(&brd->com.console, (uint8_t *)&character, 1);
#endif
}

void _init() {
  // printf("Hello World!\n");
}

void _close(int file) { (void)file; }

void _fstat(int file, void *st) {
  (void)file;
  (void)st;
}

void _isatty(int file) { (void)file; }

void _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len) {
  (void)file;

  board_t *brd = board_get_handle();

  uart_write(&brd->com.console, (uint8_t *)ptr, len);

  return len;
}

int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  return -1;
}

int _getpid() { return 1; }
