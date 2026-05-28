#include "Arduino.h"
#include "soc/usb_periph.h"
#include "driver/periph_ctrl.h"
#include "soc/gpio_reg.h"
#include "soc/rtc.h"
#include "esp_rom_gpio.h"


void __attribute__((constructor)) disable_usb_early(void) {
    periph_module_disable(PERIPH_USB_MODULE);
    periph_module_reset(PERIPH_USB_MODULE);
    
    // Pull Pins first
    esp_rom_gpio_pad_select_gpio(19);
    esp_rom_gpio_pad_select_gpio(20);

    GPIO.pin[19].pad_driver = 1;
    GPIO.pin[20].pad_driver = 1;

    PIN_PULLDWN_DIS(19);
    PIN_PULLDWN_EN(19);
    PIN_PULLDWN_DIS(20);
    PIN_PULLDWN_EN(20);
    
    // short delay to stabilize 
    delayMicroseconds(2000);
}