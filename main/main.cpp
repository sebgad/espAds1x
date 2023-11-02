#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "espAds1x.h"

float fPhyVal;

extern "C" void app_main()
{
    ADS1x esp_ads1115;
    esp_ads1115.setPhysConv(1.F,1.F);

    while(1){
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ads1115.getPhysVal(&fPhyVal));
        vTaskDelay(pdMS_TO_TICKS(250));
        ESP_LOGI(strLogTag, "Physical Value from ADS1x device: %.5f", fPhyVal);
    }
}
