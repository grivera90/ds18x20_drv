
/******************************************************************************
* Includes
*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "stdbool.h"
#include "string.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "ds18b20.h"
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS);
#define HIGH 1
#define LOW 0
#define GPIO_LED 2
#define ONEWIRE_PIN 5
/******************************************************************************
* Data types
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static const char *MODULE_NAME = "[main]";
ds18x20_t ds18b20;
uint64_t address_list[2];
uint8_t found = 0;
float temperature = 0.0;
float temperature_list[2];
/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void gpio_setup			(void);
static void setup_onewire_gpio	(uint8_t gpio, uint8_t mode);
static void ds18b20_delay_ms	(uint32_t msec);
/******************************************************************************
* app main
*******************************************************************************/
void app_main(void)
{
	gpio_setup();

	memset(&ds18b20, 0, sizeof(ds18x20_t));

	/* config the ds18b20 struct */
	ds18b20.one_wire.gpio_pin 		= ONEWIRE_PIN;
	ds18b20.one_wire.gpio_set_level = gpio_set_level;
	ds18b20.one_wire.gpio_get_level = gpio_get_level;
	ds18b20.one_wire.gpio_setup 	= setup_onewire_gpio;
	ds18b20.one_wire.delay_us		= ets_delay_us;			// in freeRTOS, you should use another function, or not....test.
	ds18b20.delay_ms 				= ds18b20_delay_ms;
	/* this isn´t necesary */
	gpio_set_pull_mode(ONEWIRE_PIN, GPIO_PULLUP_ONLY);

	int ret = ds18x20_scan_devices(&ds18b20, address_list, 1, &found);
	ESP_LOGD(MODULE_NAME, "ds18b20 founds: %d, ret: %d, address: %08x%08x", found, ret, (uint32_t)(ds18b20.address >> 32), (uint32_t)(ds18b20.address) );

    while (true) {

        gpio_set_level(GPIO_LED, HIGH);
        delay_ms(500);
        gpio_set_level(GPIO_LED, LOW);
        delay_ms(500);

        ds18b20_measure_and_read(&ds18b20);
        ESP_LOGD(MODULE_NAME, "temperature: %f, ds18x20.temp: %f", temperature, ds18b20.temp);
    }
}
/******************************************************************************
* functions definitions
*******************************************************************************/
static void gpio_setup(void)
{
    gpio_config_t io_conf;

	/* led */
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_LED);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_LED, HIGH);
}

static void setup_onewire_gpio(uint8_t gpio, uint8_t mode)
{
    gpio_set_direction(gpio, mode ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
}

static void ds18b20_delay_ms(uint32_t msec)
{
	vTaskDelay(((msec) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
	//vTaskDelay(msec / portTICK_PERIOD_MS);
}
