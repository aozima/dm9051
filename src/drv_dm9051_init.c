
#include <stdint.h>
#include <stdlib.h>

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

extern int dm9051_probe(const char *spi_dev_name, const char *device_name, int rst_pin, int int_pin);

static int dm9051_auto_init(void)
{
    dm9051_probe(DM9051_SPI_DEVICE, DM9051_DEVICE_NAME, DM9051_RST_PIN, DM9051_INT_PIN);

    return 0;
}
//INIT_ENV_EXPORT(dm9051_auto_init);

#include <finsh.h>
MSH_CMD_EXPORT(dm9051_auto_init, dm9051_auto_init);
