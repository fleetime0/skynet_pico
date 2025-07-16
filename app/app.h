#ifndef APP_H
#define APP_H

#define WIFI_SSID "skynet"
#define WIFI_PASSWORD "zxc147258369"

#define NODE_NAME "skynet_robot"
#define AGENT_IP "10.42.0.1"
#define AGENT_PORT 8888

void app_init(void);

void app_start_freertos(void);

#endif // APP_H
