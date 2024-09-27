#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "soc/adc_channel.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <esp_sleep.h>

/* Dummy stub functions*/
bool setUpDone() {
    return 0;
}
void startSetup() {
    return;
}
bool successTransmission() {
    return 0;
}
void retryTransmission() {
    return;
}

/* ENUMS for FSM states */
typedef enum {
    DEEP_SLEEP_STATE, /* System will be put to deep sleep mode when this state is detected */
    SHOWER_STATE,     /* Water is actively flowing */
    SHOWER_INTERVAL_STATE, /* System has not detected water flow for 30 seconds. Will go into light sleep mode. Use RTC to calculate amount of time in that mode. After 5 minutes wakeup from light sleep*/
    TRANSMIT_STATE,   /* System will scan for know wifi access points. And on connection, send data packet then go to DEEP_SLEEP_MODE. On failure, go to light sleep */
    SETUP_STATE,      /* System will allow user to add device to app. Go to Deep Sleep mode after success */
} FINITE_STATES;


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
#define DEFAULT_VREF        1100  // (Millivolts) Default reference voltage
#define VOLTAGE_TO_TEMP(thermistorResistance) (63.9 * exp(-0.185 * thermistorResistance) ) /* Returns temp in C inputs in kiloOhms*/


/* ------------------ Task Stack Sizes and Priorities -------------------- */
#define STACK_SIZE_TEMP_MONITOR 2000                /* (Bytes) Stack size for temperature monitoring task */
#define STACK_SIZE_TIME_MONITOR 2000                /* (Bytes) Stack size for time monitoring task */
#define STACK_SIZE_STATE_MANAGER 1000               /* (Bytes) Stack size for state monitoring task*/
#define TEMP_MONITOR_PRIORITY   10                  /* (Celsius) Task priority for temperature monitoring */
#define STATE_MANAGER_PRIORITY 10                   /* Task priority for state managment*/
#define TIME_MONITOR_PRIORITY   10                  /* (Seconds) Task priority for time monitoring */

/* ------------------ Task Names ----------------------------------------- */
#define TASK_NAME_TEMP_MONITOR  "tempMonitor"       /* Task name for temperature monitoring */
#define TASK_NAME_TIME_MONITOR  "timeMonitor"       /* Task name for time monitoring */
#define FSM_STATE_MANAGER "stateManager"            /* Task name for managing FSM States*/

/* Struct for data point used in graphing temperature vs time */
typedef struct {
    uint16_t time; // (Seconds)
    uint8_t temp; // (Celsius)
} dp;

#define DEBUG 1
#define PULSE_HYSTERISIS 5 /* Ensures random propellor movment does not register as water flow*/
#define MAX_TIME_NO_WATER_FLOW 150 /* Maximum time water flow can be off before changeing to Tramist state*/
#define MAX_NUM_TRIES_TRANSMIT 3 /*Maximum number of tries device takes to send data*/

/* Task Handels */

/* Create a task to manage the FSM state*/
TaskHandle_t stateManager = NULL;

/* Create task to monitor temperature every second */
TaskHandle_t tempMonitorHandle = NULL;

/* Create task to track water usage time */
TaskHandle_t timeMonitorHandle = NULL;

/* ------------------ Global Variables ----------------------------------- */
static volatile uint32_t totalPulses = 0;           /* Total number of pulses from the water flow sensor */
static volatile double averageWaterTemp;            /* Average water temperature in Fahrenheit */
static volatile uint32_t waterTempSamples = 0;      /* Number of water temperature samples collected */
static volatile uint32_t waterRunningTime = 0;     /* Total time the water has been running in seconds */
static TickType_t currShowerSessionTime = (TickType_t)NULL;    /* Total time since first water flow instance detected */
static volatile dp temp_graph[MAX_LENGTH_SHOWER / 10]; // Data points every 10 seconds for temperature
static uint16_t temp_graph_index = 0;

FINITE_STATES currState = SETUP_STATE; /* Holds theSHOWER_STATE current state of the system*/



/* ------------------ ISR and Task Functions ----------------------------- */

/**
 * @brief ISR for water flow sensor.
 * @param arg Pointer to the ISR argument.
 */
void IRAM_ATTR waterFlowISR(void* arg) {
    totalPulses++; /* Increment total pulses when water flow is detected */
    currState = SHOWER_STATE;
}

/**
 * @brief This task will set the State of the system
 * @param Pointer to task paramaeter (NULL)
 */
void stateManagerTask(void* parameter) {
    /* Static var to hold last pulse count (will be used later to see if there was change) */
    static uint32_t lastPulseCount = 0;
    static uint8_t transmissionTryCount = 0;
    static TickType_t recent_showerTimeStamp = (TickType_t)NULL;

    for (;;)
    {
        if (currState == SETUP_STATE && setUpDone()) {
            currState = DEEP_SLEEP_STATE;
        } else {
            startSetup();
        }

        if (currState == SHOWER_STATE) {

            /* Resume Tasks*/
            vTaskResume(tempMonitorHandle);
            vTaskResume(timeMonitorHandle);

            /* If there was water flow since last time task ran "reset" timer. Else check if timer is under MAX_TIME_NO_WATER_FLOW. If greater, go to next state and calculate entire shower session*/
            if (lastPulseCount - PULSE_HYSTERISIS >= totalPulses) {
                recent_showerTimeStamp = xTaskGetTickCount();
                lastPulseCount = totalPulses;
            } else if ((pdTICKS_TO_MS(xTaskGetTickCount() - recent_showerTimeStamp) / 1000) >= MAX_TIME_NO_WATER_FLOW) {
                currShowerSessionTime = (pdTICKS_TO_MS(xTaskGetTickCount() - currShowerSessionTime) / 1000) - MAX_TIME_NO_WATER_FLOW;
                currState = TRANSMIT_STATE;

                /* Suspend for now */
                vTaskSuspend(timeMonitorHandle);
                vTaskSuspend(tempMonitorHandle);
            }
        }

        if (currState == TRANSMIT_STATE) {
            if (successTransmission() == 1) {
                currState = DEEP_SLEEP_STATE;
            } else if (transmissionTryCount > MAX_NUM_TRIES_TRANSMIT) {
                retryTransmission();
            } else {
                currState = DEEP_SLEEP_STATE;
            }
        }

        if (currState == DEEP_SLEEP_STATE) {
            bool level = gpio_get_level(HALL_EFFECT_SENSOR);

            esp_err_t ret;

            /* If level is 0, then set trigger wake up to ESP_GPIO_WAKEUP_GPIO_HIGH, else put it to ESP_GPIO_WAKEUP_GPIO_LOW */
            if (level) {
                ret = esp_deep_sleep_enable_gpio_wakeup(1ULL << HALL_EFFECT_SENSOR, ESP_GPIO_WAKEUP_GPIO_LOW);

            } else {
                ret = esp_deep_sleep_enable_gpio_wakeup(1ULL << HALL_EFFECT_SENSOR , ESP_GPIO_WAKEUP_GPIO_HIGH);

            }

            if (ret == ESP_ERR_INVALID_ARG) {
                printf("Invalid gpio arg for deep sleep wakeup\n");
            } else if (ret == ESP_ERR_INVALID_STATE) {
                printf("Invalid gpio state for deep sleep\n");
            }

            /* Go into deep sleep */
            esp_deep_sleep_start();
        }
    }

    vTaskDelete(NULL);  // Ensure the task exits after finishing


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
    /* If showre is not running no need to run*/
    if (currState != SHOWER_STATE) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

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
        printf("Shower Time Seconds: %lu\n", waterRunningTime);
        printf("Current Water Temp %f\n\n\n\n", getTemp(holdsCalValues));
        printf("Average Water Temp %f\n\n\n\n", averageWaterTemp);
#endif

        /* Increment sample count */ //(used for averageing)
        waterTempSamples++;

        /* Block task for 1 second */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




/**
 * @brief Task for monitoring the time water has been running.
 * @param parameter Pointer to the task parameter.
 */
void timeMonitorThread(void* parameter) {
    /* If showre is not running no need to run*/
    if (currState != SHOWER_STATE) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    static TickType_t lastTick = 0;                  /* Stores the last timestamp */
    static uint32_t lastPulseCount = 0;              /* Stores pulse count from the last iteration */
    currShowerSessionTime = xTaskGetTickCount();      /* Store tick count so shower session can be calculated after */

    for (;;) {

        /* First time task is running*/
        if (lastTick != 0) {
            uint32_t localTotalPulse = totalPulses; /* Create a local copy of totalPulses */

            /* Check if pulses have increased since the last iteration (with hyterisis offet) */
            if (localTotalPulse - PULSE_HYSTERISIS > lastPulseCount) {

                TickType_t currTick = xTaskGetTickCount(); /* Get the current tick count */
                TickType_t elapsedTicks = currTick - lastTick; /* Calculate elapsed time */
                waterRunningTime += pdTICKS_TO_MS(elapsedTicks) / 1000; /* Convert ticks to seconds */
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
// void update_graph(uint16_t seconds, uint8_t temperature){
//     /* If showre is not running no need to run*/
//     if (currState != SHOWER_STATE) {
//          vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

//     dp current_data_point;
//     current_data_point.temp = temperature;
//     current_data_point.time = seconds;
//     // Circular buffer in case it
//     temp_graph[temp_graph_index++ % (MAX_LENGTH_SHOWER / 10)] = current_data_point;
// }

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
    if (gpio_isr_handler_add(HALL_EFFECT_SENSOR, waterFlowISR, NULL) != ESP_OK) {
        perror("Error attaching to waterFlowISR");
    }

   
    xTaskCreate( stateManagerTask, FSM_STATE_MANAGER, STACK_SIZE_STATE_MANAGER, NULL, STATE_MANAGER_PRIORITY, &stateManager );

    if ( stateManager == NULL ) {
        perror("Failed to create stateManger Task\n");
    }

   
    xTaskCreate(tempMonitorTask, TASK_NAME_TEMP_MONITOR, STACK_SIZE_TEMP_MONITOR, NULL, TEMP_MONITOR_PRIORITY, &tempMonitorHandle);

    if (tempMonitorHandle == NULL) {
        perror("Failed to create tempMonitorTask");
    }

    /* Suspend for now */
    vTaskSuspend(tempMonitorHandle);

    xTaskCreate(timeMonitorThread, TASK_NAME_TIME_MONITOR, STACK_SIZE_TIME_MONITOR, NULL, TIME_MONITOR_PRIORITY, &timeMonitorHandle);

    if (timeMonitorHandle == NULL) {
        perror("Failed to create timeMonitorTask");
    }

    /* Suspend for now */
    vTaskSuspend(timeMonitorHandle);

    // Suspend the main task indefinitely
    vTaskSuspend(NULL);
}

