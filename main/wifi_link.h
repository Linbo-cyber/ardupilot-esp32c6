#pragma once
#include <stdint.h>

void wifi_link_init(void);
int wifi_link_receive(uint8_t *buf, int max_len);
void wifi_link_send(const uint8_t *buf, int len);
