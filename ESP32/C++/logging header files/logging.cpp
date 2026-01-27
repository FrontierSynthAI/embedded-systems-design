#include "esp_utils.hpp"
#include "esp_log.h"

void errorLogHelper(esp_err_t err, const char *operation, const char *TAG) {
	if (err == ESP_OK) return;
	ESP_LOGE(TAG, "%s failed: %s (0x%x)", operation, esp_err_to_name(err), err);
}
