#ifndef SKYNET_BSP_H
#define SKYNET_BSP_H

#include "bsp_beep.h"
#include "bsp_key.h"
#include "bsp_led.h"

#define WIFI_SSID "OpenWrt"
#define WIFI_PASSWORD "zxc147258369"

#define VERSION_MAJOR 0x01
#define VERSION_MINOR 0x00
#define VERSION_PATCH 0x00

void bsp_init(void);

#endif // SKYNET_BSP_H
