#ifndef SHELL_HEADER_H
#define SHELL_HEADER_H

#include <microshell.h>
#include <string.h>

#define MOCOS_VERSION_MAJOR 0
#define MOCOS_VERSION_MINOR 1

// stringify helper - use xstr() to convert #define to a usable string
#define str(s) # s
#define xstr(s) str(s)

const char *shell_head =
"\r\n"
USH_SHELL_FONT_COLOR_RED
"·································································\r\n"
":                                         ███████     █████████ :\r\n"
":                                       ███░░░░░███  ███░░░░░███:\r\n"
": █████████████    ██████   ██████     ███     ░░███░███    ░░░ :\r\n"
":░░███░░███░░███  ███░░███ ███░░███   ░███      ░███░░█████████ :\r\n"
": ░███ ░███ ░███ ░███ ░███░███ ░░░    ░███      ░███ ░░░░░░░░███:\r\n"
": ░███ ░███ ░███ ░███ ░███░███  ███   ░░███     ███  ███    ░███:\r\n"
": █████░███ █████░░██████ ░░██████     ░░░███████░  ░░█████████ :\r\n"
":░░░░░ ░░░ ░░░░░  ░░░░░░   ░░░░░░        ░░░░░░░     ░░░░░░░░░  :\r\n"
"·································································\r\n"
xstr(MOCOS_VERSION_MAJOR) "." xstr(MOCOS_VERSION_MINOR) // add version number to header
"\r\n"
"\r\n"
USH_SHELL_FONT_STYLE_RESET;

#endif // SHELL_HEADER_H