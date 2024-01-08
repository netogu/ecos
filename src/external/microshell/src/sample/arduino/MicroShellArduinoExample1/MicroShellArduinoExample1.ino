
#include "microshell.h"
#include "msconf.h"

static MICROSHELL ms;

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
}

void loop()
{
    char buf[MSCONF_MAX_INPUT_LENGTH];
    Serial.print("MicroShell>");
    microshell_getline(&ms, buf, sizeof(buf));
    Serial.print("Your input text is '");
    Serial.print(buf);
    Serial.println("'");
}

