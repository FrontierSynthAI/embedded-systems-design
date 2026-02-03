#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_utils.hpp"

static const char *TAG = "MCP3008";

class MCP3008 {

    private:
        spi_host_device_t host_;
        spi_device_handle_t dev_ = nullptr;

    public:
        MCP3008(spi_host_device_t host,
                gpio_num_t pin_mosi,
                gpio_num_t pin_miso,
                gpio_num_t pin_sclk,
                gpio_num_t pin_cs,
                int clk_hz = 1 * 1000 * 1000)
            : host_(host) {

            // --- SPI bus config ---
            spi_bus_config_t buscfg = {};
            buscfg.mosi_io_num = pin_mosi;
            buscfg.miso_io_num = pin_miso;
            buscfg.sclk_io_num = pin_sclk;
            buscfg.quadwp_io_num = -1;
            buscfg.quadhd_io_num = -1;
            buscfg.max_transfer_sz = 3;

            // If you initialize the same bus elsewhere, this may return ESP_ERR_INVALID_STATE.
            // You can either ignore that specific error or ensure bus init happens once.
            errorLogHelper(spi_bus_initialize(host_, &buscfg, SPI_DMA_DISABLED),"spi init",TAG);

            // --- SPI device config ---
            spi_device_interface_config_t devcfg = {};
            devcfg.mode = 0;                 // MCP3008: SPI mode 0
            devcfg.clock_speed_hz = clk_hz;  // e.g. 1 MHz
            devcfg.spics_io_num = pin_cs;    // HW-controlled CS
            devcfg.queue_size = 1;

            errorLogHelper(spi_bus_add_device(host_, &devcfg, &dev_),"spi bus setup", TAG);
        }

        int readRaw(uint8_t channel) {
            if (channel > 7) return -1;

            // MCP3008 single-ended read:
            // TX: [0x01, 0x80 | (channel<<4), 0x00]
            // RX: data in rx[1] low 2 bits + rx[2]
            uint8_t tx[3] = {
                0x01,
                static_cast<uint8_t>(0x80 | (channel << 4)),
                0x00
            };
            uint8_t rx[3] = {0, 0, 0};

            spi_transaction_t t = {};
            t.length = 3 * 8;
            t.tx_buffer = tx;
            t.rx_buffer = rx;

            errorLogHelper(spi_device_polling_transmit(dev_, &t),"spi polling",TAG);

            return ((rx[1] & 0x03) << 8) | rx[2];
        }

        float readVolts(uint8_t channel, float vref = 3.3f) {
            int raw = readRaw(channel);
            if (raw < 0) return -1.0f;
            return (raw * vref) / 1023.0f;
        }

};


extern "C" void app_main(void) {
    // Example HSPI pin set (common on many ESP32 dev boards)
    constexpr gpio_num_t PIN_MOSI = GPIO_NUM_13;
    constexpr gpio_num_t PIN_MISO = GPIO_NUM_12;
    constexpr gpio_num_t PIN_SCLK = GPIO_NUM_14;
    constexpr gpio_num_t PIN_CS   = GPIO_NUM_15;

    // On classic ESP32: SPI2_HOST is typically HSPI. (SPI3_HOST is typically VSPI.)
    MCP3008 adc(SPI2_HOST, PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, 1'000'000);

    while (true) {
        int raw = adc.readRaw(0);
        float v = adc.readVolts(0, 3.3f);

        ESP_LOGI(TAG, "CH0 raw=%d  volts=%.3fV", raw, v);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
