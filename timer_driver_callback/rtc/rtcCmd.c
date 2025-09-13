#include "dis_dfe8219_board.h"
#include "rtcDriver.h"


static void rtcUsage(){
    printf("Usage:\n");
    printf("dfe rtc disable_irq <rtc number>\n");
    printf("dfe rtc enable_irq <rtc number>\n");
}

void rtcCmd(int argc, char *argv[])
{
    char *cmd = argv[2];

    if (argc != 4) {
        rtcUsage();
        return;
    }

    if(!strcmp(cmd, "disable_irq")) {
        if(strcmp(argv[3], "0") == 0) {
            rtc_disable_irq(0);
        } else if(strcmp(argv[3], "1") == 0) {
            rtc_disable_irq(1);
        } else {
            printf("invalid rtc number\n");
            return;
        }
    }
    else if(!strcmp(cmd, "enable_irq")) {
        if(strcmp(argv[3], "0") == 0) {
            rtc_enable_irq(0);
        } else if(strcmp(argv[3], "1") == 0) {
            rtc_enable_irq(1);
        } else {
            printf("invalid rtc number\n");
            return;
        }
    }
}

void rtcCmdInit()
{
    cmd_t rtc_cmds = {"rtc", rtcCmd};
    dis_dfe8219_register_cmds(&rtc_cmds, 1);
}