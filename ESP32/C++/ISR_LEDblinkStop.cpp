#include <cstdint>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_utils.hpp"

const static char *TAG = "LED blink";

// ---- External Button config ----
static constexpr gpio_num_t BUTTON_PIN = GPIO_NUM_22;   // external button on GPIO 22
static volatile bool g_button_pressed = false;

// ISR: minimal, ISR-safe
static void IRAM_ATTR button_isr_handler(void *arg) {
    const auto pin = static_cast<gpio_num_t>(reinterpret_cast<intptr_t>(arg));
    // Active-low: pressed when level == 0
    g_button_pressed = (gpio_get_level(pin) == 0);
}

class LEDblink {
private:
    gpio_num_t ledPin1;
    bool ledGlowState;

    void ledSetup() {
        gpio_config_t config = {
            .pin_bit_mask = 1ULL << static_cast<std::int16_t>(ledPin1),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        errorLogHelper(gpio_config(&config), "led setup", TAG);
        errorLogHelper(gpio_set_level(ledPin1, ledGlowState ? 1 : 0), "set led level", TAG);
    }

public:
    LEDblink(gpio_num_t ledPin1, bool ledGlowState)
        : ledPin1(ledPin1), ledGlowState(ledGlowState) {
        ledSetup();
    }

    void blinkLED() {
        ledGlowState = !ledGlowState;
        errorLogHelper(gpio_set_level(ledPin1, ledGlowState ? 1 : 0), "toggle led", TAG);
    }

    void setLED(bool on) {
        ledGlowState = on;
        errorLogHelper(gpio_set_level(ledPin1, on ? 1 : 0), "set led", TAG);
    }
};

static void button_setup() {
    gpio_config_t btn_cfg = {};
    btn_cfg.pin_bit_mask = 1ULL << static_cast<int>(BUTTON_PIN);
    btn_cfg.mode = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;     // internal pull-up
    btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_cfg.intr_type = GPIO_INTR_ANYEDGE;       // press + release
    errorLogHelper(gpio_config(&btn_cfg), "button setup", TAG);

    // Install ISR service once
    errorLogHelper(gpio_install_isr_service(0), "install isr service", TAG);

    // Register ISR handler
    errorLogHelper(
        gpio_isr_handler_add(
            BUTTON_PIN,
            button_isr_handler,
            reinterpret_cast<void*>(static_cast<intptr_t>(BUTTON_PIN))
        ),
        "add isr handler",
        TAG
    );

    // Set initial state
    g_button_pressed = (gpio_get_level(BUTTON_PIN) == 0);
}

extern "C" void app_main() {
    gpio_num_t ledPin1 = GPIO_NUM_23;
    bool ledGlowState = false;
    LEDblink led(ledPin1, ledGlowState);

    button_setup();

    while (true) {
        if (g_button_pressed) {
            // While button is pressed → stop blinking
            led.setLED(false);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Normal blinking
        led.blinkLED();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
