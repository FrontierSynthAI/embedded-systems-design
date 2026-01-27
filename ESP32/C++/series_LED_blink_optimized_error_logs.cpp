#include <cstdint>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <array>

#include "esp_utils.hpp"

const static char *TAG = "THREE LEDs TOGGLE";

class LEDsToggle {

	private:
		std::array<gpio_num_t,3U> pinsArray;
		gpio_num_t ledRED;
		gpio_num_t ledBLUE;
		gpio_num_t ledYELLOW;
		bool ledStateRED;
		bool ledStateBLUE;
		bool ledStateYELLOW;

		std::uint64_t makeMask() const {
			std::uint64_t mask = 0;
			for(const auto pin:pinsArray) {
				mask |= 1ULL << static_cast<std::uint16_t>(pin);
			}
			return mask;
		};

		void LEDsSetup() {
			gpio_config_t config = {
				.pin_bit_mask = makeMask(),
				.mode = GPIO_MODE_OUTPUT,
				.pull_up_en = GPIO_PULLUP_DISABLE,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.intr_type = GPIO_INTR_DISABLE
			};

			
		    errorLogHelper(gpio_config(&config),"leds config",TAG);

			errorLogHelper(gpio_set_level(pinsArray[0], ledStateRED ? 1 : 0),"set LED RED level",TAG);
			errorLogHelper(gpio_set_level(pinsArray[1], ledStateBLUE ? 1 : 0),"set LED BLUE level",TAG);
			errorLogHelper(gpio_set_level(pinsArray[2], ledStateYELLOW ? 1 : 0),"set LED YELLOW level",TAG);

		}

	public:
		LEDsToggle(const std::array<gpio_num_t,3U> &pinsArray, bool ledStateRED, bool ledStateBLUE, bool ledStateYELLOW):
		pinsArray(pinsArray),      
		ledRED(pinsArray[0]),       
		ledBLUE(pinsArray[1]), 
		ledYELLOW(pinsArray[2]),     
		ledStateRED(ledStateRED),  
		ledStateBLUE(ledStateBLUE),
		ledStateYELLOW(ledStateYELLOW)  
		{
			LEDsSetup();
		}
				
		void toggleLEDred() {
			ledStateRED = !ledStateRED;
			errorLogHelper(gpio_set_level(pinsArray[0],ledStateRED?1:0),"RED LED STATE",TAG);
			
		}

		void toggleLEDblue() {
			ledStateBLUE = !ledStateBLUE;
			errorLogHelper(gpio_set_level(pinsArray[1],ledStateBLUE?1:0),"BLUE LED STATE",TAG);
		}

		void toggleLEDyellow() {
			ledStateYELLOW = !ledStateYELLOW;
			errorLogHelper(gpio_set_level(pinsArray[2],ledStateYELLOW?1:0),"YELLOW LED STATE",TAG);
		}

	
};

extern "C" void app_main(void)
{
	auto constexpr pinsArray = std::array<gpio_num_t,3U>{GPIO_NUM_23,GPIO_NUM_22,GPIO_NUM_1};
	LEDsToggle ledsController(pinsArray,false,false,false);

	while(true) {
		ledsController.toggleLEDblue();
		vTaskDelay(pdMS_TO_TICKS(1000));
		ledsController.toggleLEDblue();

		ledsController.toggleLEDred();
		vTaskDelay(pdMS_TO_TICKS(1000));
		ledsController.toggleLEDred();

		ledsController.toggleLEDyellow();
		vTaskDelay(pdMS_TO_TICKS(1000));
		ledsController.toggleLEDyellow();
	}

}

