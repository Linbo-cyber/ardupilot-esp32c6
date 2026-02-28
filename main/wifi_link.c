#include "wifi_link.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <string.h>

static int udp_sock = -1;
static struct sockaddr_in remote_addr;
static int has_remote = 0;

void wifi_link_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASS,
            .channel = WIFI_AP_CHANNEL,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();

    // UDP socket
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_UDP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    bind(udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    // non-blocking
    int flags = fcntl(udp_sock, F_GETFL, 0);
    fcntl(udp_sock, F_SETFL, flags | O_NONBLOCK);
}

int wifi_link_receive(uint8_t *buf, int max_len) {
    socklen_t addr_len = sizeof(remote_addr);
    int n = recvfrom(udp_sock, buf, max_len, 0, (struct sockaddr *)&remote_addr, &addr_len);
    if (n > 0) has_remote = 1;
    return n;
}

void wifi_link_send(const uint8_t *buf, int len) {
    if (!has_remote || udp_sock < 0) return;
    sendto(udp_sock, buf, len, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
}
