#include "ti_msp_dl_config.h"
#include "adc_app.h"
#include "ad603.h"
#include "ad9910_app.h"
#include <stdio.h>

int main()
{
	SYSCFG_DL_init();
	NUEDC_HAL_Init();
	AD9910_AppInit();
	AD9910_AppOutputSine(1000,16383);
        while (1)
		{
            __WFI();
        }
}
