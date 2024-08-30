#ifndef SHELL_HEADER_H
#define SHELL_HEADER_H

#include <microshell.h>
#include <string.h>

#define ECO_VERSION_MAJOR 0
#define ECO_VERSION_MINOR 1

// stringify helper - use xstr() to convert #define to a usable string
#define str(s) # s
#define xstr(s) str(s)

const char *shell_head =
"\r\n"
USH_SHELL_FONT_COLOR_RED
".-----------------------------.\r\n"
"|                             |\r\n"
"|                             |\r\n"
"|                             |\r\n"
"|             8nnn88 8nnnn8   |\r\n"
"|   eeee eeee 8    8 8        |\r\n"
"|   8    8  8 8    8 8eeeee   |\r\n"
"|   8eee 8e   8    8     88   |\r\n"
"|   88   88   8    8 e   88   |\r\n"
"|   88ee 88e8 8eeee8 8eee88   |\r\n"
"|                             |\r\n"
"|                             |\r\n"
"|                             |\r\n"
"'-----------------------------'\r\n"
xstr(ECO_VERSION_MAJOR) "." xstr(ECO_VERSION_MINOR) // add version number to header
"\r\n"
"\r\n"
USH_SHELL_FONT_STYLE_RESET;

#endif // SHELL_HEADER_H