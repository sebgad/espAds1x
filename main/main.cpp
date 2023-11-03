#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"


#include "espAds1x.h"

#define ESP_INTR_FLAG_DEFAULT 0

float fPhyVal;
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR ads1x_alert_rdy_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


static void ads1x_measurement_task(void* arg)
{
  uint32_t io_num;
  ADS1x esp_ads1115;
  esp_ads1115.setPhysConv(1.F,1.F);

  for (;;) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ads1115.getPhysVal(&fPhyVal));
        printf("GPIO[%ld] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
    }
  }
}

extern "C" void app_main()
{
  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  //start gpio task
  xTaskCreate(ads1x_measurement_task, "ads1x_measurement_task", 2048, NULL, 10, NULL);

  //hook isr handler for specific gpio pin
  gpio_isr_handler_add((gpio_num_t)ADS1x_ALERTRDY_PIN, ads1x_alert_rdy_handler, (void*) ADS1x_ALERTRDY_PIN);

  while(1){
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(strLogTag, "Physical Value from ADS1x device: %.5f", fPhyVal);
  }
}
