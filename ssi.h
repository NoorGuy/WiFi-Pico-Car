#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include <stdio.h>  // for snprintf

// Make sure current_latitude and current_longitude are declared extern here
extern volatile float current_latitude;
extern volatile float current_longitude;

// Add new SSI tags "latitude" and "longitude"
const char * ssi_tags[] = {"volt", "temp", "led", "lat", "lon"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
  size_t printed = 0;
  switch (iIndex) {
  case 0: // volt
    {
      const float voltage = adc_read() * 3.3f / (1 << 12);
      printed = snprintf(pcInsert, iInsertLen, "%.3f", voltage);
    }
    break;
  case 1: // temp
    {
      const float voltage = adc_read() * 3.3f / (1 << 12);
      const float tempC = 27.0f - (voltage - 0.706f) / 0.001721f;
      printed = snprintf(pcInsert, iInsertLen, "%.2f", tempC);
    }
    break;
  case 2: // led
    {
      bool led_status = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
      printed = snprintf(pcInsert, iInsertLen, "%s", led_status ? "ON" : "OFF");
    }
    break;
  case 3: // lat
    printed = snprintf(pcInsert, iInsertLen, "%.6f", current_latitude);
    break;
  case 4: // lon
    printed = snprintf(pcInsert, iInsertLen, "%.6f", current_longitude);
    break;
  }
  return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init() {
  // Initialise ADC (internal pin)
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
