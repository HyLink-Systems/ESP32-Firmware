#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "soc/adc_channel.h"
#include "esp_adc_cal.h"
#include <math.h>

/* ------------------ Pin and Peripheral Configurations ------------------ */
#define THERMISTOR_CHANNEL      (ADC1_CHANNEL_0)    /* ADC channel for thermistor GPIO */ 
#define GPIO_OUT_VOLT           3.291                 /* (Volts) GPIO output voltage for thermistor circuit */
#define HALL_EFFECT_SENSOR     GPIO_NUM_5 

/* ------------------ Water Flow Sensor Configurations ------------------- */
#define PULSE_LITER_NUM         567                 /* (Pulses/Liter) Pulses per liter of water from the flow sensor */
#define MAX_LITERS              75.7                /* (Liter) Maximum allowable liters of water */
#define LITERS_USED (totalPulses / (float)PULSE_LITER_NUM) /* Calculation of liters used */
#define MAX_LENGTH_SHOWER 7200 // (Seconds)

/* ------------------ Thermistor Circuit Configurations ------------------ */
#define RESISTANCE_FIXED        0.817                /* (Ohm) Fixed resistor value in the thermistor voltage divider circuit */
#define THERMISTOR_RESISTANCE(adcVoltageReading) ((RESISTANCE_FIXED * adcVoltageReading) / (GPIO_OUT_VOLT - adcVoltageReading)) /* Thermistor resistance calculation */
#define DEFAULT_VREF 1100  // (Millivolts) Default reference voltage
#define VOLTAGE_TO_TEMP(thermistorResistance) (63.9 * exp(-0.185 * thermistorResistance) ) /* Returns temp in C inputs in kiloOhms*/


/* ------------------ Task Stack Sizes and Priorities -------------------- */
#define STACK_SIZE_TEMP_MONITOR 2000                /* (Bytes) Stack size for temperature monitoring task */
#define STACK_SIZE_TIME_MONITOR 2000                /* (Bytes) Stack size for time monitoring task */

#define TEMP_MONITOR_PRIORITY   10                  /* (Celsius) Task priority for temperature monitoring */
#define TIME_MONITOR_PRIORITY   10                  /* (Seconds) Task priority for time monitoring */

/* ------------------ Task Names ----------------------------------------- */
#define TASK_NAME_TEMP_MONITOR  "tempMonitor"       /* Task name for temperature monitoring */
#define TASK_NAME_TIME_MONITOR  "timeMonitor"       /* Task name for time monitoring */


/* Struct for data point used in graphing temperature vs time */
typedef struct{
    uint16_t time; // (Seconds)
    uint8_t temp; // (Celsius)
}dp;
#define DEBUG 1

/* ------------------ Global Variables ----------------------------------- */
static volatile uint32_t totalPulses = 0;           /* Total number of pulses from the water flow sensor */
static volatile double averageWaterTemp;            /* Average water temperature in Fahrenheit */
static volatile uint32_t waterTempSamples = 0;      /* Number of water temperature samples collected */
static volatile uint32_t showerTimeSeconds = 0;     /* Total time the water has been running in seconds */
static volatile dp temp_graph[MAX_LENGTH_SHOWER / 10]; // Data points every 10 seconds for temperature
static uint16_t temp_graph_index = 0;


/* ------------------ ISR and Task Functions ----------------------------- */

/**
 * @brief ISR for water flow sensor.
 * @param arg Pointer to the ISR argument.
 */
void IRAM_ATTR waterFlowISR(void* arg) {
    totalPulses++; /* Increment total pulses when water flow is detected */
}

/**
 * @brief Function to read temperature.
 * @param adcChars Pointer to the calibration characteristics.
 * @return Current temperature value in mV.
 */
float getTemp(esp_adc_cal_characteristics_t* adcChars) {
    uint32_t mV_reading = 0;

retry:
    /* Get Voltage at ADC1 */
    esp_err_t ret = esp_adc_cal_get_voltage(THERMISTOR_CHANNEL, adcChars, &mV_reading);
    
    // VOLTAGE_TO_TEMP
    if (ret == ESP_ERR_INVALID_STATE) {
        goto retry;
    } else if (ret == ESP_ERR_INVALID_ARG) {
        perror("Invalid arguments for esp_adc_cal_get_voltage");
        exit(-1);
    }
    
    float therm_resistance = THERMISTOR_RESISTANCE(mV_reading/1000.0);
    float temp = VOLTAGE_TO_TEMP(therm_resistance);
    
    #ifdef DEBUG
    printf("mV is: %lu\n", (mV_reading) );
    printf("Resistance is: %f kOhms\n", therm_resistance);
    printf("Temp is %f deg celsius\n", temp);
    #endif
    
    return temp; /* Return the voltage value for now, replace with temperature */
}

/**
 * @brief Task for monitoring temperature.
 * @param parameter Pointer to the task parameter.
 */
void tempMonitorTask(void* parameter) {
    /* Configure the ADC channel */
    if (adc1_config_channel_atten(THERMISTOR_CHANNEL, ADC_ATTEN_DB_11) != ESP_OK) {
        perror("Failed to configure ADC");
        
    }

    if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK) {
        perror("ADC config width error");
       
    }

    /* Check if ADC is calibrated */
    esp_err_t retEfuse = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (retEfuse == ESP_ERR_NOT_SUPPORTED) {
        perror("ADC not calibrated in efuse");
        
    } else if (retEfuse == ESP_ERR_INVALID_ARG) {
        printf("invalid arguments\n");
    }
    /* Allocate memory for calibration values */
    esp_adc_cal_characteristics_t* holdsCalValues = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if (holdsCalValues == NULL) {
        perror("Failed to allocate memory for holdsCalValues");
        
    }

    /* Get ADC characteristics */
       esp_adc_cal_value_t retCalVal = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, holdsCalValues);
    if (retCalVal == ESP_ADC_CAL_VAL_DEFAULT_VREF) {
        printf("Using default Vref: %d mV\n", DEFAULT_VREF);
    }

    printf("before for loop\n");
    for (;;) {
        /* First sample initialization */
        if (waterTempSamples == 0) {
            averageWaterTemp = getTemp(holdsCalValues);
        } else {
            /* Incremental average calculation */
            averageWaterTemp = averageWaterTemp + ((getTemp(holdsCalValues) - averageWaterTemp) / waterTempSamples);
        }
        #ifdef DEBUG
         printf("Total Pulses: %lu\n", totalPulses);
         printf("Shower Time Seconds: %lu\n", showerTimeSeconds);
         printf("Current Water Temp %f\n\n\n\n", getTemp(holdsCalValues));
         printf("Average Water Temp %f\n\n\n\n", averageWaterTemp);
        #endif
        

        waterTempSamples++; /* Increment sample count */

        /* Block task for 1 second */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Task for monitoring the time water has been running.
 * @param parameter Pointer to the task parameter.
 */
void timeMonitorThread(void* parameter) {
    static TickType_t lastTick = 0;           /* Stores the last timestamp */
    static uint32_t lastPulseCount = 0;       /* Stores pulse count from the last iteration */

    for (;;) {
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

        /* Block task for 1 second */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void update_graph(uint16_t seconds, uint8_t temperature){
    dp current_data_point;
    current_data_point.temp = temperature;
    current_data_point.time = seconds;
    // Circular buffer in case it 
    temp_graph[temp_graph_index++ % (MAX_LENGTH_SHOWER / 10)] = current_data_point;
}

/**
 * @brief Application entry point. Initializes ISRs and tasks.
 */
void app_main() {
    /* Configure GPIO 1 as interrupt on rising edge for water flow sensor */
    const gpio_config_t gpio1_struct = {
        .pin_bit_mask = 1ULL << HALL_EFFECT_SENSOR,    /* Pin 1 */
        .mode = GPIO_MODE_INPUT,               /* Set pin 1 as input */
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
    if (gpio_isr_handler_add(HALL_EFFECT_SENSOR, waterFlowISR, (void*) HALL_EFFECT_SENSOR) != ESP_OK) {
        perror("Error attaching to waterFlowISR");
    }

    /* Create task to monitor temperature every second */
    TaskHandle_t tempMonitorHandle = NULL;
    xTaskCreate(tempMonitorTask, TASK_NAME_TEMP_MONITOR, STACK_SIZE_TEMP_MONITOR, NULL, TEMP_MONITOR_PRIORITY, &tempMonitorHandle);

     if (tempMonitorHandle == NULL) {
         perror("Failed to create tempMonitorTask");
         vTaskDelete(tempMonitorHandle);
     }

     /* Create task to track water usage time */
     TaskHandle_t timeMonitorHandle = NULL;
     xTaskCreate(timeMonitorThread, TASK_NAME_TIME_MONITOR, STACK_SIZE_TIME_MONITOR, NULL, TIME_MONITOR_PRIORITY, &timeMonitorHandle);

     if (timeMonitorHandle == NULL) {
         vTaskDelete(timeMonitorHandle);
     }

     // Suspend the main task indefinitely
    vTaskSuspend(NULL);
}

