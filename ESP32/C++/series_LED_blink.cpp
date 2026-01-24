#include <cstdint>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <array>

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

		void errorLogHelper(esp_err_t err, const char *operation) {
			for(const auto pin: pinsArray) {
				ESP_LOGE(TAG,"GPIO %d: %s failed: %s (0x%x)", static_cast<std::uint32_t>(pin), operation,esp_err_to_name(err),err);
			}
		}

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

			esp_err_t err = gpio_config(&config);
			if(err != ESP_OK) {
				errorLogHelper(err,"leds config");
			}

			esp_err_t errLevel1 = gpio_set_level(pinsArray[0], ledStateRED ? 1 : 0);
			esp_err_t errLevel2 = gpio_set_level(pinsArray[1], ledStateBLUE ? 1 : 0);
			esp_err_t errLevel3 = gpio_set_level(pinsArray[2], ledStateYELLOW ? 1 : 0);

			if (errLevel1 != ESP_OK) {
				errorLogHelper(errLevel1, "set LED RED level");
			}
			if (errLevel2 != ESP_OK) {
				errorLogHelper(errLevel2, "set LED BLUE level");
			};

			if (errLevel2 != ESP_OK) {
				errorLogHelper(errLevel3, "set LED YELLOW level");
			};
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
			esp_err_t redLEDerr = gpio_set_level(pinsArray[0],ledStateRED?1:0);
			if(redLEDerr != ESP_OK) {
				errorLogHelper(redLEDerr,"toggle red led");
			}
		}

		void toggleLEDblue() {
			ledStateBLUE = !ledStateBLUE;
			esp_err_t blueLEDerr = gpio_set_level(pinsArray[1],ledStateBLUE?1:0);
			if(blueLEDerr != ESP_OK) {
				errorLogHelper(blueLEDerr,"toggle blue led");
			}
		}

		void toggleLEDyellow() {
			ledStateYELLOW = !ledStateYELLOW;
			esp_err_t yellowLEDerr = gpio_set_level(pinsArray[2],ledStateYELLOW?1:0);
			if(yellowLEDerr != ESP_OK) {
				errorLogHelper(yellowLEDerr,"toggle yellow led");
			}
		}

	
};

extern "C" void app_main(void)
{
	auto constexpr pinsArray = std::array<gpio_num_t,3U>{GPIO_NUM_23,GPIO_NUM_22,GPIO_NUM_1};
	LEDsToggle ledsController(pinsArray,false,false,false);

	while(true) {
		ledsController.toggleLEDblue();
		vTaskDelay(pdMS_TO_TICKS(300));
		ledsController.toggleLEDblue();

		ledsController.toggleLEDred();
		vTaskDelay(pdMS_TO_TICKS(300));
		ledsController.toggleLEDred();

		ledsController.toggleLEDyellow();
		vTaskDelay(pdMS_TO_TICKS(300));
		ledsController.toggleLEDyellow();
	}

}

