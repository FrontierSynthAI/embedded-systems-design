#include <string>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_utils.hpp"

static const char* TAG = "UART_EXAMPLE";

// UART configuration
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        GPIO_NUM_17
#define UART_RX_PIN        GPIO_NUM_16
#define UART_BUF_SIZE      1024

class UARTHandler {
public:
    UARTHandler(uart_port_t port = UART_PORT_NUM) : uart_port(port) {
        initUART();
    }

    void sendData(const std::string &data) {
        uart_write_bytes(uart_port, data.c_str(), data.length());
    }

    std::string readData() {
        uint8_t data[UART_BUF_SIZE];
        int len = uart_read_bytes(uart_port, data, sizeof(data) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            return std::string((char*)data);
        }
        return "";
    }

private:
    uart_port_t uart_port;

    void initUART() {
    	uart_config_t uart_config = {
			.baud_rate = UART_BAUD_RATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		};
		errorLogHelper(uart_param_config(uart_port, &uart_config), "UART param config", TAG);

		errorLogHelper(uart_set_pin(uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "UART set pin", TAG);
		errorLogHelper(uart_driver_install(uart_port, UART_BUF_SIZE * 2, 0, 0, nullptr, 0), "UART driver install", TAG);

    	ESP_LOGI(TAG, "UART initialized on port %d", uart_port);
    }
};

extern "C" void app_main() {
    UARTHandler uart;

    uart.sendData("Hello from ESP32!\n");

    while (true) {
        std::string received = uart.readData();
        if (!received.empty()) {
            ESP_LOGI(TAG, "Received: %s", received.c_str());
        }
    }
}
