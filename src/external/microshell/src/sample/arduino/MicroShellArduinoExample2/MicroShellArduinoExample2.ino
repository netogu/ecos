
#include "microshell.h"
#include "mscmd.h"
#include "msconf.h"

typedef struct {
  int dummy;
} USER_OBJECT;

static MICROSHELL ms;
static MSCMD mscmd;
static USER_OBJECT usrobj = {
    .dummy = 10,
};

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
    Serial.write(&c, 1);
}

static char urx(void)
{
    char c;
    while (!Serial.available()) {
    }
    Serial.readBytes(&c, 1);
    return c;
}

static void action_hook(MSCORE_ACTION action)
{
}

void setup()
{
    Serial.begin(9600);
    microshell_init(&ms, utx, urx, action_hook);
    mscmd_init(&mscmd, table, sizeof(table) / sizeof(table[0]), &usrobj);
}

void loop()
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    MSCMD_USER_RESULT r;
    Serial.print("MicroShell>");
    microshell_getline(&ms, buf, sizeof(buf));
    mscmd_execute(&mscmd, buf, &r);
}

static MSCMD_USER_RESULT usrcmd_system(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    int i;
    Serial.println("[SYSTEM]");
    msopt_get_argc(msopt, &argc);
    for (i = 0; i < argc; i++) {
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        Serial.println(buf);
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_config(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    int i;
    Serial.println("[CONFIG]");
    msopt_get_argc(msopt, &argc);
    for (i = 0; i < argc; i++) {
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        Serial.println(buf);
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_help(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    Serial.print(
            "system : system command\r\n"
            "config : config command\r\n"
            "help   : help command\r\n"
            );
    return 0;
}

