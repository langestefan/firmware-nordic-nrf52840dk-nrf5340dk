#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

extern bool ei_ble_rcv_cmd_flag;
extern char ei_ble_rcv_cmd_buffer[CONFIG_BT_NUS_UART_BUFFER_SIZE];

void ble_nus_init(void);
void ble_nus_send_data(const char *buffer, uint8_t size);