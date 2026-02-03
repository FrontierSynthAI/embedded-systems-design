#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_utils.hpp"

static const char *TAG = "ADC_RAW";

class AdcRaw {
	
	private:
		adc_unit_t unit_;
		adc_channel_t channel_;
		adc_oneshot_unit_handle_t adc_handle_{nullptr};

	public:
		AdcRaw(adc_unit_t unit, adc_channel_t channel)
			: unit_(unit), channel_(channel) {

			adc_oneshot_unit_init_cfg_t unit_cfg = {
				.unit_id  = unit_,
				.ulp_mode = ADC_ULP_MODE_DISABLE,
			};

			errorLogHelper(
				adc_oneshot_new_unit(&unit_cfg, &adc_handle_),
				"adc oneshot new unit",
				TAG
			);

			adc_oneshot_chan_cfg_t chan_cfg = {
				.atten    = ADC_ATTEN_DB_12,
				.bitwidth = ADC_BITWIDTH_12
			};

			errorLogHelper(
				adc_oneshot_config_channel(adc_handle_, channel_, &chan_cfg),
				"adc config channel",
				TAG
			);
		}

		~AdcRaw() {
			if (adc_handle_) {
				errorLogHelper(
					adc_oneshot_del_unit(adc_handle_),
					"adc delete unit",
					TAG
				);
			}
		}

		// expose handle + channel (no logic)
		adc_oneshot_unit_handle_t handle() const { 
			return adc_handle_; 
		}

		adc_channel_t channel() const { 
			return channel_; 
		}

};

extern "C" void app_main(void) {

    constexpr adc_unit_t    ADC_UNIT    = ADC_UNIT_1;
    constexpr adc_channel_t ADC_CHANNEL = ADC_CHANNEL_0; // GPIO36

    AdcRaw adc(ADC_UNIT, ADC_CHANNEL);

    while (true) {
        int raw = 0;

        errorLogHelper(
            adc_oneshot_read(adc.handle(), adc.channel(), &raw),
            "adc oneshot read",
            TAG
        );

        ESP_LOGI(TAG, "ADC raw value: %d", raw);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
