#include <string>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_utils.hpp"

static const char* TAG = "UART_EXAMPLE";

static constexpr size_t UART_BUF_SIZE = 1024;
static constexpr int DEFAULT_BAUD_RATE = 115200;

class UARTHandler {
    private:
        uart_port_t uartPort;
        gpio_num_t txPin;
        gpio_num_t rxPin;

        void initUart() {
            uart_config_t uartConfig = {
                .baud_rate = DEFAULT_BAUD_RATE,  // use global constant
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };

            errorLogHelper(uart_param_config(uartPort, &uartConfig), "UART param config", TAG);
            errorLogHelper(uart_set_pin(uartPort, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "UART set pin", TAG);
            errorLogHelper(uart_driver_install(uartPort, UART_BUF_SIZE * 2, 0, 0, nullptr, 0), "UART driver install", TAG);

            ESP_LOGI(TAG, "UART initialized on port %d", uartPort);
        }

    public:
        // Constructor: pass only the port and pins
        UARTHandler(uart_port_t port, gpio_num_t tx, gpio_num_t rx):uartPort(port), txPin(tx), rxPin(rx) {
            initUart();
        }

        void sendData(const std::string &data) {
            uart_write_bytes(uartPort, data.c_str(), data.length());
        }

        std::string readData() {
            uint8_t dataBuffer[UART_BUF_SIZE];
            int len = uart_read_bytes(uartPort, dataBuffer, sizeof(dataBuffer) - 1, pdMS_TO_TICKS(1000));
            if (len > 0) {
                dataBuffer[len] = '\0';
                return std::string((char*)dataBuffer);
            }
            return "";
        }
};

extern "C" void app_main() {
    // Only pass port and pins — baud rate is fixed by global constant
    UARTHandler uart(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);

    uart.sendData("Hello from ESP32!\n");

    while (true) {
        std::string received = uart.readData();
        if (!received.empty()) {
            ESP_LOGI(TAG, "Received: %s", received.c_str());
        }
    }
}
