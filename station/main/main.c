#include "station_example_main.h"
#include <esp_sleep.h>
#include <math.h>
#include <stdio.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/adc_channel.h"

/* Stub functions for demonstration purposes */
bool setUpDone() { return false; }
void startSetup() {}
bool successTransmission() { return false; }
void retryTransmission() {}

/* Finite State Machine (FSM) states */
typedef enum {
  DEEP_SLEEP_STATE,
  SHOWER_STATE,
  POST_SHOWER_PROCESSING_STATE,
  TRANSMIT_STATE,
  SETUP_STATE
} FINITE_STATES;

/* Pin and Peripheral Configurations */
#define THERMISTOR_CHANNEL (ADC1_CHANNEL_0)
#define GPIO_OUT_VOLT 3.291
#define HALL_EFFECT_SENSOR GPIO_NUM_5

/* Water Flow Sensor Configurations */
#define PULSE_LITER_NUM 567
#define MAX_LITERS 75.7
#define PULSE_TO_LITER(pulseCount) (pulseCount / (float)PULSE_LITER_NUM)
#define MAX_LENGTH_SHOWER 7200

/* Thermistor Circuit Configurations */
#define RESISTANCE_FIXED 0.817
#define THERMISTOR_RESISTANCE(adcVoltageReading) \
  ((RESISTANCE_FIXED * adcVoltageReading) / (GPIO_OUT_VOLT - adcVoltageReading))
#define DEFAULT_VREF 1100
#define VOLTAGE_TO_TEMP(thermistorResistance) \
  (63.9 * exp(-0.185 * thermistorResistance))

/* Task Configurations */
#define STACK_SIZE_TEMP_MONITOR 2000
#define STACK_SIZE_TIME_MONITOR 2000
#define LOGGER_STACK_SIZE 2000
#define STACK_SIZE_STATE_MANAGER 4000
#define TEMP_MONITOR_PRIORITY 10
#define STATE_MANAGER_PRIORITY 10
#define TIME_MONITOR_PRIORITY 10
#define LOGGER_PRIORITY 10

/* Task Names */
#define TASK_NAME_TEMP_MONITOR "tempMonitor"
#define TASK_NAME_TIME_MONITOR "timeMonitor"
#define FSM_STATE_MANAGER "stateManager"
#define LOGGER  "logger"

/* Data Structure for Temp/WaterRate vs Time */
typedef struct {
  uint16_t time;
  uint8_t temp;
  float currWaterRate;
} __attribute__((packed)) dataPoint;

#define DEBUG 1
#define PULSE_HYSTERISIS 5
#define MAX_TIME_NO_WATER_FLOW_ALLOWED_MS (60 * 1000)
#define MAX_NUM_TRIES_TRANSMIT 3
#define temp_graph_length 540

/* Task Handles */
static TaskHandle_t stateManager = NULL;
static TaskHandle_t timeMonitorHandle = NULL;
static TaskHandle_t loggerHandle = NULL;
/* Global System Data Structure */
typedef struct {
  uint32_t totalPulses;
  float averageWaterTemp;
  float currWaterTemp;
  float litersUsed;
  uint32_t waterTempSamples;
  uint32_t waterRunningTime;
  TickType_t showerSessionStartTime;
  FINITE_STATES currState;
  uint32_t currShowerSessionTime;
  uint16_t temp_graph_index;
  esp_adc_cal_characteristics_t* holdsCalValues;
  float currWaterRate;
  dataPoint temp_graph[temp_graph_length];
} SystemData;

static volatile SystemData systemData = {0};

typedef struct {
    float litersUsed;
    float averageWaterTemp;
    uint32_t waterRunningTime;
    uint32_t currShowerSessionTime;
} __attribute__((packed)) TX_PACKET;


/* Counting and Binary Semaphores */
SemaphoreHandle_t xSemaphoreCounting;
SemaphoreHandle_t xSemaphoreWaterFlowing1;
SemaphoreHandle_t xSemaphoreWaterFlowing2;

/* ISR for Water Flow Sensor */
void IRAM_ATTR waterFlowISR(void* arg) {
  //printf("ISR Triggered at Line: %d\n", __LINE__);
  xSemaphoreGive(xSemaphoreCounting);
  xSemaphoreGive(xSemaphoreWaterFlowing1);
  xSemaphoreGive(xSemaphoreWaterFlowing2);
}

/* FSM State Handlers */
void handleSetupState(void);
void handleShowerState(void);
void handleTransmitState(void);
void handleDeepSleepState(void);
void handlePostShowerState(void);

float getTemp(esp_adc_cal_characteristics_t* calVals);

/* FSM State Manager Task */
void stateManagerTask(void* parameter) {
  printf("Entering stateManagerTask at Line: %d\n", __LINE__);
  systemData.currState = SHOWER_STATE;
  for (;;) {
    printf("Current State: %d at Line: %d\n", systemData.currState, __LINE__);
    switch (systemData.currState) {
      case SETUP_STATE:
        handleSetupState();
        break;
      case SHOWER_STATE:
        handleShowerState();
        break;
      case POST_SHOWER_PROCESSING_STATE:
        handlePostShowerState();
        break;
      case TRANSMIT_STATE:
        handleTransmitState();
        break;
      case DEEP_SLEEP_STATE:
        handleDeepSleepState();
        break;
      default:
        printf("Unknown State at Line: %d\n", __LINE__);
        break;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void printSystemData() {
    printf("\n\n\nTotal Pulses: %lu\n", systemData.totalPulses);
    //printf("Average Water Temp: %.2f\n", systemData.averageWaterTemp);
    printf("Current Water Temp: %.2f\n", systemData.currWaterTemp);
    // printf("Water Temp Samples: %lu\n", systemData.waterTempSamples);
    printf("Water Running Time: %lu\n", systemData.waterRunningTime);
    printf("Shower Session Start Time: %lu\n", systemData.showerSessionStartTime);
    printf("Current State: %d\n", systemData.currState);
    printf("Current Shower Session Time: %lu\n", systemData.currShowerSessionTime);
    printf("Water Rate: %.2f\n\n\n\n\n", systemData.currWaterRate);  
    printf("Liters Used: %f\n", systemData.litersUsed);
}


float calcAverage();

float calcAverage() {
    float average = 0.0;
    int count = 0;

    // Iterate over each entry in the temp_graph
    for (int i = 0; i < systemData.temp_graph_index; i++) {
        dataPoint entry = systemData.temp_graph[i];

        // Only consider entries where water rate is non-zero
        if (entry.currWaterRate > 0) {
            count++;
            // Update average using incremental averaging formula
            average += (entry.temp - average) / count;
        }
    }

    return average;
}

void printTemperatureGraphWithAverage() {
    printf("Temperature Graph:\n");
    for (int i = 0; i < systemData.temp_graph_index; i++) {
        dataPoint entry = systemData.temp_graph[i];
        

        printf("Index %d - Time: %u, Temp: %u, Water Rate: %.2f\n",
               i, entry.time, entry.temp, entry.currWaterRate);
    
        
    }

    printSystemData();

}


void handlePostShowerState() {
    /* turn off ISR*/

    systemData.averageWaterTemp = calcAverage();
    printf("Average Water Temp: %.2f\n", systemData.averageWaterTemp);

    /*subtract MAX_TIME_NO_WATER_FLOW_ALLOWED_MS from currShowerSessionTime*/
    systemData.currShowerSessionTime -= (MAX_TIME_NO_WATER_FLOW_ALLOWED_MS / 1000);
    systemData.currState = TRANSMIT_STATE;
    systemData.litersUsed = PULSE_TO_LITER(systemData.totalPulses);
    

    printTemperatureGraphWithAverage();
}

void handleSetupState() {

   
  printf("Handling Setup State at Line: %d\n", __LINE__);
  
  char ssid[32];
  char password[32];


    if (!get_wifi_credentials_from_nvs(ssid, 32, password, 32)) {


        bool hi = save_wifi_credentials_to_nvs("NA iPhone", "12345678");
        if (hi) {
            printf("Saved SSID and Password\n");
        } else {
            printf("Failed to save\n");
        }
    } else {
        printf("SSID: %s\n", ssid);
        printf("Password: %s\n", password);
        printf("Credentials already exist. No need to save.\n");
    }


  
}

void handleShowerState() {
  printf("Handling Shower State at Line: %d\n", __LINE__);
  vTaskSuspend(timeMonitorHandle);
  vTaskResume(timeMonitorHandle);
  if (xSemaphoreTake(xSemaphoreWaterFlowing1,
                     pdMS_TO_TICKS(MAX_TIME_NO_WATER_FLOW_ALLOWED_MS)) == pdTRUE) {
    systemData.totalPulses = uxSemaphoreGetCount(xSemaphoreCounting);
    printf("Water Flow Detected, Total Pulses: %lu at Line: %d\n",
           systemData.totalPulses, __LINE__);
  } else {
    systemData.currState = POST_SHOWER_PROCESSING_STATE;
    vTaskSuspend(timeMonitorHandle);
    vTaskSuspend(loggerHandle);
    printf("No Water Flow, Switching to POST_SHOWER_PROCESSING_STATE at Line: %d\n",
           __LINE__);
  }
}

void handleTransmitState() {

//Initialize NVS
    // ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    bool success = wifi_init_sta();

    if (success) {
        printf("Success connecting to wifi\n");
    } else {
        printf("Failed to connect\n");
    }

    vTaskDelay(5000/pdMS_TO_TICKS(1000));
    systemData.currState = DEEP_SLEEP_STATE;
}

void handleDeepSleepState() {
  bool level = gpio_get_level(HALL_EFFECT_SENSOR);

  esp_err_t ret;

  if (level) {
    ret = esp_deep_sleep_enable_gpio_wakeup(1ULL << HALL_EFFECT_SENSOR,
                                            ESP_GPIO_WAKEUP_GPIO_LOW);
    printf("Setting Deep Sleep Wakeup on Low Level at Line: %d\n", __LINE__);

  } else {
    ret = esp_deep_sleep_enable_gpio_wakeup(1ULL << HALL_EFFECT_SENSOR,
                                            ESP_GPIO_WAKEUP_GPIO_HIGH);
    printf("Setting Deep Sleep Wakeup on High Level at Line: %d\n", __LINE__);
  }

  if (ret == ESP_ERR_INVALID_ARG || ret == ESP_ERR_INVALID_STATE) {
    printf("Error setting deep sleep wakeup trigger at Line: %d\n", __LINE__);
  }

  esp_deep_sleep_start();
}

esp_adc_cal_characteristics_t* initTempMonitor(void) {
  if (adc1_config_channel_atten(THERMISTOR_CHANNEL, ADC_ATTEN_DB_11) !=
          ESP_OK ||
      adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK) {
    perror("Failed to configure ADC");

    return NULL;
  }

  esp_err_t retEfuse = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);

  if (retEfuse != ESP_OK) {
    printf("ADC calibration error at Line: %d\n", __LINE__);
  }

  esp_adc_cal_characteristics_t* calVals =
      calloc(1, sizeof(esp_adc_cal_characteristics_t));

  if (calVals == NULL) {
    perror("Memory allocation failed for calibration values");

    return NULL;
  }

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                           DEFAULT_VREF, calVals);

  return calVals;
}

void loggerThread(void* parameter) {
    static uint16_t lastPulseCount = 0;
    // printf("[Line %d] - Initializing lastPulseCount to 0.\n", __LINE__);

    systemData.showerSessionStartTime = xTaskGetTickCount();
    // printf("[Line %d] - Set shower session start time at tick %lu.\n", __LINE__, systemData.showerSessionStartTime);

    static TickType_t lastTick = 0;
    // printf("[Line %d] - Initializing lastTick to 0.\n", __LINE__);

    for (;;) {
        TickType_t currTick = xTaskGetTickCount();
        systemData.currShowerSessionTime = pdTICKS_TO_MS(currTick - systemData.showerSessionStartTime) / 1000;
        // printf("[Line %d] - Updated current shower session time: %lu seconds.\n", __LINE__, systemData.currShowerSessionTime);

        TickType_t elapsedTicks = currTick - lastTick;
        // printf("[Line %d] - Elapsed ticks: %lu.\n", __LINE__, elapsedTicks);

        /* If there was no water flow */
        if ((systemData.totalPulses - lastPulseCount) <= PULSE_HYSTERISIS) {

            systemData.currWaterRate = 0;
            dataPoint logEntry = {
                systemData.currShowerSessionTime,
                (uint8_t)NULL, /* Don't populate temp */
                systemData.currWaterRate     /* Flow was 0 */
            };
            // printf("[Line %d] - No water flow detected, created logEntry with zero flow.\n", __LINE__);
            systemData.temp_graph[systemData.temp_graph_index] = logEntry;
            // printf("[Line %d] - Updated temp_graph at index %u.\n", __LINE__, systemData.temp_graph_index);
        } else {
            systemData.currWaterRate = (float)(PULSE_TO_LITER(systemData.totalPulses) - PULSE_TO_LITER(lastPulseCount)) / (pdTICKS_TO_MS(elapsedTicks) / 1000.0 / 60.0);

            dataPoint logEntry = {
                systemData.currShowerSessionTime,
                (uint8_t)systemData.currWaterTemp,
                (float)(PULSE_TO_LITER(systemData.totalPulses) - PULSE_TO_LITER(lastPulseCount))
            };
            // printf("[Line %d] - Water flow detected, created logEntry with flow rate.\n", __LINE__);
            systemData.temp_graph[systemData.temp_graph_index] = logEntry;
            // printf("[Line %d] - Updated temp_graph at index %u.\n", __LINE__, systemData.temp_graph_index);
        }

        systemData.temp_graph_index = (systemData.temp_graph_index+1) % temp_graph_length;
        lastPulseCount = systemData.totalPulses;
        // printf("[Line %d] - Updated lastPulseCount to %u.\n", __LINE__, lastPulseCount);
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Assuming a delay is needed
        printSystemData();
    }
}

        


void timeMonitorThread(void* parameter) {
    printf("Line %d: Entering timeMonitorThread\n", __LINE__);
    static TickType_t lastTick = 0;
    printf("Line %d: Initialized lastTick\n", __LINE__);


    for (;;) {
       // printf("Line %d: Loop start\n", __LINE__);
        if (lastTick == 0) {
            
            lastTick = xTaskGetTickCount();
            // printf("Line %d: Updated lastTick in else block\n", __LINE__);

            //systemData.averageWaterTemp = getTemp(systemData.holdsCalValues);
            // printf("Line %d: Updated averageWaterTemp in else block\n", __LINE__);

            // printf("Line %d: Reset lastPulseCount in else block\n", __LINE__);
        } else {
            if (xSemaphoreTake(xSemaphoreWaterFlowing2, 0) == pdTRUE) {
                // printf("Line %d: Semaphore taken\n", __LINE__);
                TickType_t currTick = xTaskGetTickCount();
                // printf("Line %d: Current tick count obtained\n", __LINE__);

                TickType_t elapsedTicks = currTick - lastTick;
                // printf("Line %d: Calculated elapsedTicks\n", __LINE__);

                lastTick = currTick;
                // printf("Line %d: Updated lastTick\n", __LINE__);

                systemData.waterRunningTime += pdTICKS_TO_MS(elapsedTicks) / 1000;
                // printf("Line %d: Updated waterRunningTime\n", __LINE__);

                systemData.currWaterTemp = getTemp(systemData.holdsCalValues);
                // printf("Line %d: Current temperature obtained\n", __LINE__);

                // systemData.averageWaterTemp = systemData.averageWaterTemp +
                //                             ((systemData.currWaterTemp - systemData.averageWaterTemp) /
                //                             systemData.waterTempSamples);
                printf("Line %d: Updated averageWaterTemp\n", __LINE__);

                // systemData.waterTempSamples++;
                // printf("Line %d: Incremented waterTempSamples\n", __LINE__);
            }
        }



        vTaskDelay(3000 / portTICK_PERIOD_MS);
        //printf("Line %d: Delayed task by 5000 ms\n", __LINE__);
    }
}

void app_main() {

     esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // handleSetupState();
    

  printf("Line %d: in main\n", __LINE__);
  xSemaphoreCounting = xSemaphoreCreateCounting(0xFFFF, 0);
  printf("Line %d: Created counting semaphore\n", __LINE__);

  xSemaphoreWaterFlowing1 = xSemaphoreCreateBinary();
  printf("Line %d: Created binary semaphore 1\n", __LINE__);

  xSemaphoreWaterFlowing2 = xSemaphoreCreateBinary();
  printf("Line %d: Created binary semaphore 2\n", __LINE__);




  printf("Line %d: Initializing Temperature Monitor\n", __LINE__);
  systemData.holdsCalValues = initTempMonitor();

  if (systemData.holdsCalValues == NULL) {
    printf("Line %d: Initialization failed, exiting\n", __LINE__);
    return;
  }

  const gpio_config_t gpio1_struct = {
      .pin_bit_mask = 1ULL << HALL_EFFECT_SENSOR,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE};
  printf("Line %d: Configured GPIO for Hall Effect Sensor\n", __LINE__);

  BaseType_t res1 = xTaskCreate(timeMonitorThread, TASK_NAME_TIME_MONITOR,
                                STACK_SIZE_TIME_MONITOR, NULL,
                                TIME_MONITOR_PRIORITY, &timeMonitorHandle);
  printf("Line %d: Created time monitor task, result: %d\n", __LINE__, res1);
  vTaskSuspend(timeMonitorHandle);

  BaseType_t res2 =
      xTaskCreate(stateManagerTask, FSM_STATE_MANAGER, STACK_SIZE_STATE_MANAGER,
                  NULL, STATE_MANAGER_PRIORITY, &stateManager);
  printf("Line %d: Created state manager task, result: %d\n", __LINE__, res2);
  
  BaseType_t res3 = xTaskCreate(loggerThread, LOGGER, LOGGER_STACK_SIZE, NULL, LOGGER_PRIORITY, &loggerHandle );


  if (res1 == pdFAIL || res2 == pdFAIL) {
    printf("Line %d: Task creation failed\n", __LINE__);
  }


  gpio_config(&gpio1_struct);
  printf("Line %d: GPIO configuration applied\n", __LINE__);

  gpio_install_isr_service(0);
  printf("Line %d: ISR service installed\n", __LINE__);

  gpio_isr_handler_add(HALL_EFFECT_SENSOR, waterFlowISR, NULL);
  printf("Line %d: ISR handler added for Hall Effect Sensor\n", __LINE__);



  vTaskSuspend(NULL);
}

/**
 * @brief Function to read temperature.
 * @param adcChars Pointer to the calibration characteristics.
 * @return Current temperature value in mV.
 */

float getTemp(esp_adc_cal_characteristics_t* adcChars)

{
  uint32_t mV_reading = 0;

retry:

  /* Get Voltage at ADC1 */

  esp_err_t ret =
      esp_adc_cal_get_voltage(THERMISTOR_CHANNEL, adcChars, &mV_reading);

  // VOLTAGE_TO_TEMP

  if (ret == ESP_ERR_INVALID_STATE)

    goto retry;

  else if (ret == ESP_ERR_INVALID_ARG)

  {
    perror("Invalid arguments for esp_adc_cal_get_voltage");

    exit(-1);
  }

  float therm_resistance = THERMISTOR_RESISTANCE(mV_reading / 1000.0);

  float temp = VOLTAGE_TO_TEMP(therm_resistance);

#ifdef DEBUG

//   printf("mV is: %lu\n", (mV_reading));

//   printf("Resistance is: %f kOhms\n", therm_resistance);

//   printf("Temp is %f deg celsius\n", temp);

#endif

  return temp;
}