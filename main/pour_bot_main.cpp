/*
 * 1. Open up the project properties
 * 2. Visit C/C++ General > Preprocessor Include Paths, Macros, etc
 * 3. Select the Providers tab
 * 4. Check the box for "CDT GCC Built-in Compiler Settings"
 * 5. Set the compiler spec command to "xtensa-esp32-elf-gcc ${FLAGS} -E -P -v -dD "${INPUTS}""
 * 6. Rebuild the index
*/

#include <esp_log.h>
#include <string>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ds18b20.h"

static char tag[]="pour-bot";

extern "C" {
	void app_main(void);
}

const int DS_PIN = 18;

void mainTask(void *pvParameters){
  ds18b20_init(DS_PIN);

  while (1) {
    printf("Temperature: %0.1f\n",ds18b20_get_temp());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void app_main(void)
{
	xTaskCreatePinnedToCore(&mainTask, "mainTask", 2048, NULL, 5, NULL, 0);
}

