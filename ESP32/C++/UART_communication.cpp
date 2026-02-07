/*
ESP32-1 (sender)                     ESP32-2 (receiver)
+----------------+                   +----------------+
| std::string msg|                   | auto dataBuffer|
| sendData()     |                   | readData()     |
| uart_write_bytes| --TX pin cable--> | uart_read_bytes|
| TX FIFO        |                   | RX FIFO        |
+----------------+                   +----------------+
*/

#include <string>
#include <cstdint>
#include <array>

#include <driver/gpio.h>
#include <driver/uart.h>
#include<freertos/FreeRTOS.h>
#include<freertos/task.h>
#include <esp_log.h>

#include "esp_utils.hpp"


static const char* TAG = "UART example";

static constexpr std::uint32_t  BAUDERATE = 115200;
static constexpr size_t BUFFERsize = 1024;

class UARTcomm {
	private:
		uart_port_t uartPort;
		gpio_num_t txPin;
		gpio_num_t rxPin;

		void initUART() {
			uart_config_t uartConfig = {
				.baud_rate = BAUDERATE,
				.data_bits = UART_DATA_8_BITS,
				.parity = UART_PARITY_DISABLE,
				.stop_bits = UART_STOP_BITS_1,
				.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
			};
			errorLogHelper(uart_param_config(uartPort,&uartConfig),"UART param config",TAG);

			errorLogHelper(uart_set_pin(uartPort,txPin,rxPin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE),"UART set pin", TAG);
			errorLogHelper(uart_driver_install(uartPort,BUFFERsize*2,0,0,nullptr,0),"UART driver install",TAG);

			ESP_LOGI(TAG,"UART intiliazed on port %d",uartPort);
		}

	public:
		UARTcomm(uart_port_t uartPort, gpio_num_t txPin, gpio_num_t rxPin):uartPort(uartPort),txPin(txPin),rxPin(rxPin) {
			initUART();
		}

		void sendData(const std::string &data) {
			uart_write_bytes(uartPort,data.c_str(),data.length());
		}

		std::string readData() {
            auto dataBuffer = std::array<std::uint8_t,BUFFERsize>{};
            int len = uart_read_bytes(uartPort, dataBuffer.data(), sizeof(dataBuffer) - 1, pdMS_TO_TICKS(1000));
            if (len > 0) {
                dataBuffer[len] = '\0';
                return std::string(reinterpret_cast<char*>(dataBuffer.data()));
            }
            return "";
        }
};


extern "C" void app_main() {
	/* ESP32 pins
	| UART  | TX Pin   | RX Pin   | Notes               |
	| ----- | -------- | -------- | ------------------- |
	| UART0 | GPIO1    | GPIO3    | Usually USB console |
	| UART1 | Any GPIO | Any GPIO | Fully configurable  |
	| UART2 | Any GPIO | Any GPIO | Fully configurable  |


	UART1 / UART2: assignable to most GPIOs
	BUT avoid: GPIO6–11 (flash/PSRAM), input-only pins, or pins reserved by your board

	*/

	UARTcomm uart(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);

	while(true) {
		uart.sendData("Hello \n");             // send
		auto received = uart.readData();       // immediately read back
		if(!received.empty()) {
			ESP_LOGI(TAG, "Received: %s", received.c_str());
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}







