#ifndef BOARD_LOG_H
#define BOARD_LOG_H

#include <microshell.h>
#include "stm32g4/uart.h"
#include "tiny_printf.h"

// prinf macro logging in green
#define LOG_INFO(...) do { \
  printf(USH_SHELL_FONT_COLOR_GREEN); \
  printf(__VA_ARGS__); \
  printf("\r\n"); \
  printf(USH_SHELL_FONT_STYLE_RESET); \
} while(0)

#define LOG_ERROR(...) do { \
  printf(USH_SHELL_FONT_COLOR_RED); \
  printf(__VA_ARGS__); \
  printf(USH_SHELL_FONT_STYLE_RESET); \
} while(0)

#define LOG_OK(...) do { \
  printf(__VA_ARGS__); \
  printf("\t - "); \
  printf(USH_SHELL_FONT_COLOR_GREEN); \
  printf("OK\r\n"); \
  printf(USH_SHELL_FONT_STYLE_RESET); \
} while(0)

#define LOG_FAIL(...) do { \
  printf(__VA_ARGS__); \
  printf("\t - "); \
  printf(USH_SHELL_FONT_COLOR_RED); \
  printf("FAIL\r\n"); \
  printf(USH_SHELL_FONT_STYLE_RESET); \
} while(0)

#define LOG_WARN(...) do { \
  printf(USH_SHELL_FONT_COLOR_YELLOW); \
  printf(__VA_ARGS__); \
  printf(USH_SHELL_FONT_STYLE_RESET); \
} while(0)

#define LOG_CLEAR() do { \
  printf("\033[2J\033[H"); \
} while(0)

#endif // BOARD_LOG_H