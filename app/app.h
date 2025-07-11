#ifndef SKYNET_APP_H
#define SKYNET_APP_H

#define WIFI_SSID "OpenWrt"
#define WIFI_PASSWORD "zxc147258369"

#define NODE_NAME "skynet_robot"
#define AGENT_IP "192.168.3.2"
#define AGENT_PORT 8888

void app_init(void);

void app_start_freertos(void);

#endif // SKYNET_APP_H
