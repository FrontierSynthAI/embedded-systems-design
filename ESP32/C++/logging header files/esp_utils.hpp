#pragma once

#include "esp_err.h"

// Log error if err != ESP_OK
void errorLogHelper(esp_err_t err, const char* operation, const char* tag);
