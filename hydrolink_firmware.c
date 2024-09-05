#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "soc/adc_channel.h"
#include "esp_adc_cal.h"


#define PULSE_LITER_NUM     567 /* Number of pulses for every liter of water */
#define STACK_SIZE_TEMP_MONITOR 1000 /* Stack size for tempMonitorThread task */
#define TASK_NAME_TEMP_MONITOR "tempMonitor"
#define TEMP_MONITOR_PRIORITY 10
#define TASK_NAME_TIME_MONITOR "timeMonitor"
#define STACK_SIZE_TEMP_MONITOR 1000
#define TIME_MONITOR_PRIORITY 3
#define TASK_NAME_LED_CONTROLLER "ledController"
#define STACK_SIZE_LED_CONTROLLER 1000
#define LED_CONTROLLER_PRIORITY 15

#define RESISTANCE_FIXED 5000 /* Fixed resistor value of thermistor voltage divider circuit */
#define GPIO_OUT_VOLT 3.3     /* Voltage output of GPIO pin */
#define THERMISTOR_CHANNEL (ADC1_CHANNEL_0)     /* GPIO pin for thermistor */

#define MAX_LITERS 75.7 /* Maximum allowable shower gallons */
#define LITERS_USED (totalPulses / (float)PULSE_LITER_NUM)
#define WATER_FLOW_SENSOR_PIN GPIO_NUM_1

#define RED_LED_PIN     2 /* GPIO pin for Red LED */
#define GREEN_LED_PIN   3 /* GPIO pin for Green LED */
#define YELLOW_LED_PIN  4 /* GPIO pin for Yellow LED */

#define THERMISTOR_RESISTOR(adcVoltageReading) ( ( RESISTANCE_FIXED * adcVoltageReading ) / (GPIO_OUT_VOLT - adcVoltageReading) ) /* Convert Adc reading from volage divider to Thermistor resistance */

/* Holds the total number of pulses from the hall effect sensor */
static volatile uint32_t totalPulses = 0;

/* Average temperature of the shower in Fahrenheit */
static volatile double averageWaterTemp;

/* Number of samples captured of the water temperature */
static volatile uint32_t waterTempSamples = 0;

/* Total time the water was running in seconds */
static volatile uint32_t showerTimeSeconds = 0;





/** 
 * @brief ISR for water flow sensor.
 * @param arg Pointer to the ISR argument.
 */
void IRAM_ATTR waterFlowISR(void* arg) {
    totalPulses++; /* Increment total pulses when water flow is detected */
}

/** 
 * @brief Function for reading temperature (to be implemented).
 * @param Pointer to calibration function esp_adc_cal_characteristics_t
 * @return Current temperature value.
 */
float getTemp(esp_adc_cal_characteristics_t* adcChars) {
    /* Hold milivolts */
    uint32_t mV_reading = NULL;

retry:
    /* Get Voltage at ADC1*/
    esp_err_t ret = esp_adc_cal_get_voltage(THERMISTOR_CHANNEL, adcChars, &mV_reading);

    if ( ret == ESP_ERR_INVALID_STATE ) {
        goto retry;

    } else if ( ret == ESP_ERR_INVALID_ARG ) {
        perror("Invalid arguments for esp_adc_cal_get_voltage");
        exit(-1);
    } else {

        /* Just return voltage value for now*/
        return mV_reading;


    }
   
}

/** 
 * @brief Task for monitoring temperature.
 * @param parameter Pointer to the task parameter.
 */
void tempMonitorThread(void* parameter) {
    /* Configure the ADC channel */
    
    if ( adc1_config_channel_atten(THERMISTOR_CHANNEL, ADC_ATTEN_DB_11) != ESP_OK ) {
        perror("Failed to configure ADC");
        return;
    }

    if ( adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK) {
        perror("ADC config width error");
        return;
    }

    /**
     * 
     *   Ensure ADC values burned into e-fuse 
     */

    esp_err_t retEfuse = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);

    if ( retEfuse != ESP_OK ) {
        perror("ADC not calibrated in efuse");
        return;
    }

    /*Create empty data structure to hold calibration values */
    esp_adc_cal_characteristics_t* holdsCalValues = (esp_adc_cal_characteristics_t*) calloc( 1, sizeof(esp_adc_cal_characteristics_t) );

    if ( holdsCalValues == NULL ) {
        perror("Failed to allocate memory for holdsCalValues");
        free(holdsCalValues);
        return;
    }

    /* Get the characteristics of the adc */
    esp_adc_cal_value_t retCalVal = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, NULL, holdsCalValues);

    for ( ; ; ) {
        /* If it's the first sample, initialize averageWaterTemp */
        if (waterTempSamples == 0) {
            averageWaterTemp = getTemp(holdsCalValues);
        } else {
            /* Incremental average calculation */
            averageWaterTemp = averageWaterTemp + ((getTemp(holdsCalValues) - averageWaterTemp) / waterTempSamples);
        }

        waterTempSamples++; /* Increment sample count */
        
        /* Block task for 1 second */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


}

/** 
 * @brief Task for monitoring time while the water is running.
 * @param parameter Pointer to the task parameter.
 */
void timeMonitorThread(void* parameter) {
    static TickType_t lastTick = 0; /* Stores the last timestamp */
    static uint32_t lastPulseCount = 0; /* Stores pulse count from the last iteration */

    for ( ; ; ) {
        if (lastTick != 0) {
            uint32_t localTotalPulse = totalPulses; /* Create a local copy of totalPulses */

            /* Check if pulses have increased since the last iteration */
            if (localTotalPulse > lastPulseCount) {
                TickType_t currTick = xTaskGetTickCount(); /* Get the current tick count */
                TickType_t elapsedTicks = currTick - lastTick; /* Calculate elapsed time */
                showerTimeSeconds += pdTICKS_TO_MS(elapsedTicks) / 1000; /* Convert ticks to seconds */
                lastTick = currTick; /* Update lastTick */
                lastPulseCount = localTotalPulse; /* Update lastPulseCount */
            } else {
                lastTick = xTaskGetTickCount(); /* Reset lastTick if no water flow is detected */
            }
        } else {
            lastPulseCount = totalPulses; /* Initialize lastPulseCount */
            lastTick = xTaskGetTickCount(); /* Initialize lastTick */
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); /* Block task for 1 second */
    }
}

/** 
 * @brief Task for controlling LEDs based on water usage.
 * @param parameter Pointer to the task parameter.
 */
void ledControllerThread(void* parameter) {
    /* Initialize LED pins */
    if (gpio_reset_pin(RED_LED_PIN) != ESP_OK) {
        perror("RED GPIO reset error");
    }  

    if (gpio_reset_pin(YELLOW_LED_PIN) != ESP_OK) {
        perror("Yellow GPIO reset error");
    }  

    if (gpio_reset_pin(GREEN_LED_PIN) != ESP_OK) {
        perror("Green GPIO reset error");
    }

    for ( ; ; ) {
        /* Control LEDs based on liters used */
        if (LITERS_USED > 0 && LITERS_USED <= 0.78 * MAX_LITERS) {
            /* Green LED Solid */
            gpio_set_level(GREEN_LED_PIN, 1);
            gpio_set_level(RED_LED_PIN, 0);
            gpio_set_level(YELLOW_LED_PIN, 0);
        } else if (LITERS_USED > 0.78 * MAX_LITERS && LITERS_USED <= 0.80 * MAX_LITERS) {
            /* Green LED Blinking (not implemented) */
        } else if (LITERS_USED > 0.80 * MAX_LITERS && LITERS_USED <= 0.92 * MAX_LITERS) {
            /* Yellow LED Solid */
            gpio_set_level(GREEN_LED_PIN, 0);
            gpio_set_level(YELLOW_LED_PIN, 1);
            gpio_set_level(RED_LED_PIN, 0);
        } else if (LITERS_USED > 0.92 * MAX_LITERS && LITERS_USED <= 0.95 * MAX_LITERS) {
            /* Yellow LED Blinking (not implemented) */
        } else if (LITERS_USED > 0.95 * MAX_LITERS && LITERS_USED <= MAX_LITERS) {
            /* Red LED Solid */
            gpio_set_level(GREEN_LED_PIN, 0);
            gpio_set_level(YELLOW_LED_PIN, 0);
            gpio_set_level(RED_LED_PIN, 1);
        } else if (LITERS_USED > MAX_LITERS) {
            /* Red LED Blinking (not implemented) */
        } else {
            perror("Something wrong happened");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); /* Block task for half a second */
    }
}


/** 
 * @brief Application entry point. Initializes ISRs and tasks.
 */
void app_main() {
    /* Configure GPIO 1 as interrupt on rising edge for Water flow sensor */
    const gpio_config_t gpio1_struct = {
        .pin_bit_mask = 1ULL << WATER_FLOW_SENSOR_PIN, /* PIN 1 */
        .mode = GPIO_MODE_INPUT, /* Set pin 1 as input */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    if (gpio_config(&gpio1_struct) != ESP_OK) {
        perror("GPIO 1 interrupt configuring problem");
    }

    /* Initialize the ISR service */
    if (gpio_install_isr_service(0) != ESP_OK) {
        perror("Failed to initialize ISR service");
    }

    /* Attach ISR handler for water flow sensor */
    if (gpio_isr_handler_add(GPIO_NUM_1, waterFlowISR, (void*) GPIO_NUM_1) != ESP_OK) {
        perror("Error attaching to waterFlowISR");
    }

    /* Create task to monitor temperature every second */
    TaskHandle_t tempMonitorHandle = NULL;
    xTaskCreate(tempMonitorThread, TASK_NAME_TEMP_MONITOR, STACK_SIZE_TEMP_MONITOR, NULL, TEMP_MONITOR_PRIORITY, &tempMonitorHandle);

    /* Error handling for tempMonitorThread creation */
    if (tempMonitorHandle == NULL) {
        perror("Failed to create tempMonitorTask");
        vTaskDelete(tempMonitorHandle);
    }

    /* Create task to track water usage time */
    TaskHandle_t timeMonitorHandle = NULL;
    xTaskCreate(timeMonitorThread, TASK_NAME_TIME_MONITOR, STACK_SIZE_TEMP_MONITOR, NULL, TEMP_MONITOR_PRIORITY, &timeMonitorHandle);

    /* Error handling for timeMonitorThread creation */
    if (timeMonitorHandle == NULL) {
        vTaskDelete(timeMonitorHandle);
    }

    /* Create task to control LEDs */
    TaskHandle_t ledControllerHandle = NULL;
    xTaskCreate(ledControllerThread, TASK_NAME_LED_CONTROLLER, STACK_SIZE_LED_CONTROLLER, NULL, LED_CONTROLLER_PRIORITY, &ledControllerHandle);

    /* Error handling for ledControllerThread creation */
    if (ledControllerHandle == NULL) {
        perror("Failed to create ledControllerHandle");
        vTaskDelete(ledControllerHandle);
    }
}
