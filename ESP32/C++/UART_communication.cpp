#include <string>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_utils.hpp"

static const char* TAG = "UART_EXAMPLE";

// UART configuration using camelCase static constexpr
static constexpr uart_port_t uartPortNum = UART_NUM_1;
static constexpr int       uartBaudRate = 115200;
static constexpr gpio_num_t uartTxPin   = GPIO_NUM_17;
static constexpr gpio_num_t uartRxPin   = GPIO_NUM_16;
static constexpr size_t    uartBufSize  = 1024;

class UARTHandler {
    private:
        uart_port_t uartPort;

        void initUart() {
            uart_config_t uartConfig = {
                .baud_rate = uartBaudRate,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };

            errorLogHelper(uart_param_config(uartPort, &uartConfig), "UART param config", TAG);
            errorLogHelper(uart_set_pin(uartPort, uartTxPin, uartRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "UART set pin", TAG);
            errorLogHelper(uart_driver_install(uartPort, uartBufSize * 2, 0, 0, nullptr, 0), "UART driver install", TAG);

            ESP_LOGI(TAG, "UART initialized on port %d", uartPort);
        }

    public:
        // Constructor: NO default argument, NO equals sign, uses initializer list
        UARTHandler(uart_port_t port):uartPort(port) {
            initUart();
        }

        void sendData(const std::string &data) {
            uart_write_bytes(uartPort, data.c_str(), data.length());
        }

        std::string readData() {
            uint8_t dataBuffer[uartBufSize];
            int len = uart_read_bytes(uartPort, dataBuffer, sizeof(dataBuffer) - 1, pdMS_TO_TICKS(1000));
            if (len > 0) {
                dataBuffer[len] = '\0';
                return std::string((char*)dataBuffer);
            }
            return "";
        }
};

extern "C" void app_main() {
    // Must pass the UART port explicitly now
    UARTHandler uart(uartPortNum);

    uart.sendData("Hello from ESP32!\n");

    while (true) {
        std::string received = uart.readData();
        if (!received.empty()) {
            ESP_LOGI(TAG, "Received: %s", received.c_str());
        }
    }
}
