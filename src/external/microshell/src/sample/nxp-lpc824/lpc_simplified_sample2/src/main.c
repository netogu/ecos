/**
 * @file      main.c
 * @author    Shinichiro Nakamura (CuBeatSystems)
 * ===============================================================
 * MicroShell (Version 0.0.2)
 * Copyright (c) 2016, 2017 Shinichiro Nakamura (CuBeatSystems)
 * ===============================================================
 * The MIT License : https://opensource.org/licenses/MIT
 *
 * Copyright (c) 2016, 2017 Shinichiro Nakamura (CuBeatSystems)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "chip.h"
#include "uart.h"
#include "microshell.h"
#include "mscmd.h"
#include <cr_section_macros.h>

typedef struct {
    void (*puts)(char *str);
} USER_OBJECT;

static MSCMD_USER_RESULT usrcmd_system(MSOPT *msopt, MSCMD_USER_OBJECT usrobj);
static MSCMD_USER_RESULT usrcmd_config(MSOPT *msopt, MSCMD_USER_OBJECT usrobj);
static MSCMD_USER_RESULT usrcmd_help(MSOPT *msopt, MSCMD_USER_OBJECT usrobj);

static MSCMD_COMMAND_TABLE table[] = {
    {   "system",   usrcmd_system   },
    {   "config",   usrcmd_config   },
    {   "help",     usrcmd_help     },
    {   "?",        usrcmd_help     },
};

static void utx(char c)
{
    uart_putc(c);
}

static char urx(void)
{
    return uart_getc();
}

static void action_hook(MSCORE_ACTION action)
{
}

int main(void)
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    MICROSHELL ms;
    MSCMD mscmd;
    USER_OBJECT usrobj = {
        .puts = uart_puts,
    };

    SystemCoreClockUpdate();

    uart_init();
    uart_puts(
            "\r\n\r\n"
            "======================================\r\n"
            " MicroShell Simple Example for LPC824 \r\n"
            "======================================\r\n"
            );
    uart_puts(" Type 'help' for a list of commands.\r\n");

    microshell_init(&ms, utx, urx, action_hook);
    mscmd_init(&mscmd, table, sizeof(table) / sizeof(table[0]), &usrobj);

    while (1) {
        MSCMD_USER_RESULT r;
        uart_puts("MicroShell>");
        microshell_getline(&ms, buf, sizeof(buf));
        mscmd_execute(&mscmd, buf, &r);
    }

    return 0;
}

static MSCMD_USER_RESULT usrcmd_system(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    int i;
    uart_puts("[SYSTEM]\r\n");
    msopt_get_argc(msopt, &argc);
    for (i = 0; i < argc; i++) {
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        uart_puts(" '");
        uart_puts(buf);
        uart_puts("'\r\n");
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_config(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    int i;
    uart_puts("[CONFIG]\r\n");
    msopt_get_argc(msopt, &argc);
    for (i = 0; i < argc; i++) {
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        uart_puts(" '");
        uart_puts(buf);
        uart_puts("'\r\n");
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_help(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    uo->puts(
            "system : system command\r\n"
            "config : config command\r\n"
            "help   : help command\r\n"
            );
    return 0;
}

