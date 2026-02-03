#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_utils.hpp"

static const char *TAG = "MCP3008";

class MCP3008 {
private:
    spi_device_handle_t spi_;

public:
    MCP3008(spi_host_device_t host,
            gpio_num_t cs,
            gpio_num_t mosi,
            gpio_num_t miso,
            gpio_num_t sclk) {

        spi_bus_config_t buscfg = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1
        };

        errorLogHelper(
            spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED),
            "spi bus init",
            TAG
        );

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 1 * 1000 * 1000,   // 1 MHz
            .mode = 0,                           // SPI mode 0
            .spics_io_num = cs,
            .queue_size = 1
        };

        errorLogHelper(
            spi_bus_add_device(host, &devcfg, &spi_),
            "spi add device",
            TAG
        );
    }

    int readRaw(uint8_t channel) {
        if (channel > 7) return -1;

        uint8_t tx[3] = {
            0x01,
            static_cast<uint8_t>(0x80 | (channel << 4)),
            0x00
        };

        uint8_t rx[3] = {0};

        spi_transaction_t t = {};
        t.length = 3 * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;

        errorLogHelper(
            spi_device_transmit(spi_, &t),
            "spi transmit",
            TAG
        );

        int value = ((rx[1] & 0x03) << 8) | rx[2];
        return value;
    }
};

extern "C" void app_main(void) {

    MCP3008 adc(
        SPI2_HOST,      // VSPI
        GPIO_NUM_5,     // CS
        GPIO_NUM_23,    // MOSI
        GPIO_NUM_19,    // MISO
        GPIO_NUM_18     // SCLK
    );

    while (true) {
        int raw = adc.readRaw(0);   // Channel 0
        ESP_LOGI(TAG, "ADC CH0 raw: %d", raw);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
