#include "ti_msp_dl_config.h"

/* 程序入口：完成板级初始化后等待业务代码接入。 */
int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        __WFI();
    }
}
