#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

// FreeRTOS Headers
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ============================================================
// HARDWARE PIN MAPPING
// ============================================================

// External Push Button for Emergency STOP (Active LOW: 0 = Pressed, 1 = Released)
#define STOP_PORT            GPIO3
#define STOP_PIN             18U      // P3_18 (Arduino D7, Header J1 Pin 15)
#define STOP_PORT_PERIPH     PORT3

// MQ-2 Gas Sensor Digital Output (DO) (Active LOW: 0 = Gas Detected, 1 = Clean Air)
#define GAS_PORT             GPIO0
#define GAS_PIN              24U      // P0_24 (Arduino A0 socket, Header J3 Pin 2)
#define GAS_PORT_PERIPH      PORT0

// External LEDs (Header J1)
#define GREEN_LED_PORT       GPIO1
#define GREEN_LED_PIN        5U       // P1_5  (Arduino D3, J1 Pin 7)

#define YELLOW_LED_PORT      GPIO1
#define YELLOW_LED_PIN       6U       // P1_6  (Arduino D4, J1 Pin 9)

#define RED_LED_PORT         GPIO1
#define RED_LED_PIN          7U       // P1_7  (Arduino D5, J1 Pin 11)

// External Buzzer (Header J1)
#define BUZZER_PORT          GPIO3
#define BUZZER_PIN           17U      // P3_17 (Arduino D6, J1 Pin 13)

// External DHT22 Data (Header J1)
#define DHT_PORT             GPIO1
#define DHT_PIN              4U       // P1_4  (Arduino D2, J1 Pin 5)
#define DHT_PORT_PERIPH      PORT1

// Input buffer enable mask helper for MCX series
#if defined(PORT_PCR_IBE)
  #define MCX_IBE_ENABLE         PORT_PCR_IBE(1)
#elif defined(PORT_PCR_IBE_MASK)
  #define MCX_IBE_ENABLE         PORT_PCR_IBE_MASK
#else
  #define MCX_IBE_ENABLE         (1U << 8)
#endif

// ============================================================
// THRESHOLDS & TIMING DEFINITIONS
// ============================================================

// Temperature Thresholds in °C x10
#define TEMP_WARNING_X10     340        // 34.0 °C (Slightly high temp)
#define TEMP_HAZARD_X10      360        // 36.0 °C (Very high temp)

#define GAS_TASK_PERIOD_MS   50
#define ENV_TASK_PERIOD_MS   2000
#define SAFETY_TASK_PERIOD_MS 10
#define LOGGER_TASK_PERIOD_MS 1000

#define BLINK_WARNING_MS     300
#define BLINK_HAZARD_MS      300
#define BLINK_EMERGENCY_MS   100

#define DHT_TIMEOUT          200000

// System States
#define NORMAL               0
#define WARNING              1
#define HAZARD               2
#define EMERGENCY            3

// ============================================================
// DATA STRUCTURES
// ============================================================

struct SensorData {
    bool     gasDetected;
    int16_t  temperature_x10;
    int16_t  humidity_x10;
};

struct SystemStateData {
    bool     gasDetected;
    int16_t  temperature_x10;
    int16_t  humidity_x10;
    int      systemState;
};

// ============================================================
// FREERTOS OBJECTS & GLOBAL VARIABLES
// ============================================================

QueueHandle_t sensorQueue;
SemaphoreHandle_t stateMutex;

volatile bool gasDetected = false;
volatile int temperature_x10 = 290;
volatile int humidity_x10 = 700;
volatile int systemState = NORMAL;

volatile uint32_t dhtOkCount = 0;
volatile uint32_t dhtFailCount = 0;
volatile int dhtLastError = 0;

struct SystemStateData sharedState;

// ============================================================
// DHT22 LOW-LEVEL DRIVER (BIT-BANGING)
// ============================================================

static void set_dht_pin_dir(bool is_output) {
    gpio_pin_config_t pin_config = {
        is_output ? kGPIO_DigitalOutput : kGPIO_DigitalInput,
        1
    };
    GPIO_PinInit(DHT_PORT, DHT_PIN, &pin_config);
    if (!is_output) {
        DHT_PORT_PERIPH->PCR[DHT_PIN] = PORT_PCR_MUX(0) | MCX_IBE_ENABLE | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    }
}

static uint32_t dhtPulse(uint8_t level) {
    uint32_t count = 0;
    while (GPIO_PinRead(DHT_PORT, DHT_PIN) == level) {
        if (++count >= DHT_TIMEOUT) {
            return DHT_TIMEOUT;
        }
    }
    return count;
}

static int dhtReadFrame(uint8_t data[5]) {
    set_dht_pin_dir(false);

    if (dhtPulse(1) == DHT_TIMEOUT) return 1;
    if (dhtPulse(0) == DHT_TIMEOUT) return 2;
    if (dhtPulse(1) == DHT_TIMEOUT) return 3;

    for (int i = 0; i < 40; i++) {
        uint32_t low = dhtPulse(0);
        uint32_t high = dhtPulse(1);

        if (low == DHT_TIMEOUT) return 4;
        if (high == DHT_TIMEOUT) return 5;

        if (high > low) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    return 0;
}

bool readDHT22(int *temp_x10, int *hum_x10) {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    set_dht_pin_dir(true);
    GPIO_PinWrite(DHT_PORT, DHT_PIN, 0);
    SDK_DelayAtLeastUs(18000, SystemCoreClock);

    set_dht_pin_dir(false);
    GPIO_PinWrite(DHT_PORT, DHT_PIN, 1);
    SDK_DelayAtLeastUs(40, SystemCoreClock);

    taskENTER_CRITICAL();
    int err = dhtReadFrame(data);
    taskEXIT_CRITICAL();

    if (err != 0) {
        dhtLastError = err;
        return false;
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        dhtLastError = 6;
        return false;
    }

    *hum_x10 = (int16_t)((data[0] << 8) | data[1]);
    int16_t rawTemp = (int16_t)((data[2] << 8) | data[3]);

    if (rawTemp & 0x8000) {
        rawTemp &= 0x7FFF;
        *temp_x10 = -rawTemp;
    } else {
        *temp_x10 = rawTemp;
    }

    return true;
}

// ============================================================
// RTOS TASKS
// ============================================================

void GasTask(void *parameter) {
    while (1) {
        // MQ-2 DO is active LOW: 0 = Gas Detected, 1 = Clean Air
        bool isDetected = (GPIO_PinRead(GAS_PORT, GAS_PIN) == 0);
        gasDetected = isDetected;

        struct SensorData data;
        data.gasDetected = isDetected;
        data.temperature_x10 = temperature_x10;
        data.humidity_x10 = humidity_x10;

        xQueueSend(sensorQueue, &data, 0);

        vTaskDelay(pdMS_TO_TICKS(GAS_TASK_PERIOD_MS));
    }
}

void EnvironmentTask(void *parameter) {
    int consecutiveFails = 0;

    while (1) {
        int temp = 0;
        int hum = 0;
        bool ok = false;

        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            ok = readDHT22(&temp, &hum);
            if (!ok) {
                dhtFailCount++;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        if (ok) {
            temperature_x10 = temp;
            humidity_x10 = hum;
            dhtOkCount++;
            consecutiveFails = 0;

            struct SensorData data;
            data.gasDetected = gasDetected;
            data.temperature_x10 = temp;
            data.humidity_x10 = hum;

            xQueueSend(sensorQueue, &data, 0);
        } else {
            consecutiveFails++;
            if (consecutiveFails >= 5) {
                temperature_x10 = 0;
                humidity_x10 = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ENV_TASK_PERIOD_MS));
    }
}

void SafetyTask(void *parameter) {
    struct SensorData data;

    while (1) {
        while (xQueueReceive(sensorQueue, &data, 0) == pdPASS) {
            gasDetected = data.gasDetected;
            temperature_x10 = data.temperature_x10;
            humidity_x10 = data.humidity_x10;
        }

        bool gasAlarm = gasDetected;
        int temp = temperature_x10;
        int hum = humidity_x10;

        // Button is active LOW: 0 = Pressed, 1 = Released
        bool emergency = (GPIO_PinRead(STOP_PORT, STOP_PIN) == 0);

        int wanted;
        if (emergency) {
            // 1. External Button Pressed -> EMERGENCY
            wanted = EMERGENCY;
        } else if (gasAlarm || temp >= TEMP_HAZARD_X10) {
            // 2. Gas Detected OR Temperature >= 36.0°C -> HAZARD
            wanted = HAZARD;
        } else if (temp >= TEMP_WARNING_X10) {
            // 3. Temperature >= 34.0°C -> WARNING
            wanted = WARNING;
        } else {
            // 4. Normal condition
            wanted = NORMAL;
        }

        systemState = wanted;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            sharedState.gasDetected = gasAlarm;
            sharedState.temperature_x10 = temp;
            sharedState.humidity_x10 = hum;
            sharedState.systemState = wanted;
            xSemaphoreGive(stateMutex);
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool green = false;
        bool yellow = false;
        bool red = false;
        bool buzzer = false;

        switch (wanted) {
            case NORMAL:
                green = true;     // Green LED solid ON
                yellow = false;
                red = false;
                buzzer = false;   // Buzzer silent
                break;

            case WARNING:
                green = false;
                yellow = ((now / BLINK_WARNING_MS) % 2) == 0; // Yellow LED blinks
                red = false;
                buzzer = false;   // Buzzer silent
                break;

            case HAZARD:
                green = false;
                yellow = false;
                red = ((now / BLINK_HAZARD_MS) % 2) == 0;      // Red LED blinks
                buzzer = true;    // Buzzer ON
                break;

            case EMERGENCY:
                green = false;
                yellow = false;
                red = ((now / BLINK_EMERGENCY_MS) % 2) == 0;   // Red LED blinks rapidly
                buzzer = true;    // Buzzer ON
                break;
        }

        GPIO_PinWrite(GREEN_LED_PORT, GREEN_LED_PIN, green ? 1 : 0);
        GPIO_PinWrite(YELLOW_LED_PORT, YELLOW_LED_PIN, yellow ? 1 : 0);
        GPIO_PinWrite(RED_LED_PORT, RED_LED_PIN, red ? 1 : 0);
        GPIO_PinWrite(BUZZER_PORT, BUZZER_PIN, buzzer ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}

void printDecimal(const char* label, int32_t value_x10, const char* unit) {
    PRINTF("%s", label);
    if (value_x10 < 0) {
        PRINTF("-");
        value_x10 = -value_x10;
    }
    PRINTF("%d.%d%s\r\n", (int)(value_x10 / 10), (int)(value_x10 % 10), unit);
}

void LoggerTask(void *parameter) {
    while (1) {
        struct SystemStateData localState;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localState = sharedState;
            xSemaphoreGive(stateMutex);
        }

        PRINTF("\r\n--------------------------------\r\n");
        PRINTF(" INDUSTRIAL SAFETY MONITOR\r\n");
        PRINTF("--------------------------------\r\n");

        PRINTF("Gas Sensor (DO): %s\r\n", localState.gasDetected ? "GAS DETECTED (TRIGGERED)" : "CLEAN (NORMAL)");
        printDecimal("Temperature    : ", localState.temperature_x10, " C");
        printDecimal("Humidity       : ", localState.humidity_x10, " %");

        PRINTF("- - - - - - - - - - - - - - - - \r\n");
        PRINTF("Gas Status     : %s\r\n", localState.gasDetected ? "HAZARD" : "NORMAL");
        PRINTF("Temp Status    : %s\r\n", (localState.temperature_x10 >= TEMP_HAZARD_X10) ? "HAZARD" : ((localState.temperature_x10 >= TEMP_WARNING_X10) ? "WARNING" : "NORMAL"));
        PRINTF("Hum Status     : %s\r\n", "NORMAL");
        PRINTF("- - - - - - - - - - - - - - - - \r\n");

        PRINTF("SYSTEM STATUS  : ");
        if (localState.systemState == EMERGENCY) {
            PRINTF("EMERGENCY (EXTERNAL SW PRESSED)\r\n");
        } else if (localState.systemState == HAZARD) {
            PRINTF("HAZARD\r\n");
        } else if (localState.systemState == WARNING) {
            PRINTF("WARNING\r\n");
        } else {
            PRINTF("NORMAL\r\n");
        }

        PRINTF("DHT22 reads    : OK %d / FAIL %d\r\n", dhtOkCount, dhtFailCount);
        PRINTF("--------------------------------\r\n");

        vTaskDelay(pdMS_TO_TICKS(LOGGER_TASK_PERIOD_MS));
    }
}

// ============================================================
// MAIN ENTRY POINT
// ============================================================

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    // Enable Clocks for GPIO and PORT blocks
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Port3);

    // 1. Output Pins (LEDs & Buzzer)
    gpio_pin_config_t out_config = {
        kGPIO_DigitalOutput,
        0
    };
    GPIO_PinInit(GREEN_LED_PORT, GREEN_LED_PIN, &out_config);
    GPIO_PinInit(YELLOW_LED_PORT, YELLOW_LED_PIN, &out_config);
    GPIO_PinInit(RED_LED_PORT, RED_LED_PIN, &out_config);
    GPIO_PinInit(BUZZER_PORT, BUZZER_PIN, &out_config);

    // Initial state: Green ON, others OFF
    GPIO_PinWrite(GREEN_LED_PORT, GREEN_LED_PIN, 1);
    GPIO_PinWrite(YELLOW_LED_PORT, YELLOW_LED_PIN, 0);
    GPIO_PinWrite(RED_LED_PORT, RED_LED_PIN, 0);
    GPIO_PinWrite(BUZZER_PORT, BUZZER_PIN, 0);

    // 2. Input Pins (External Button, Gas DO, DHT22)
    gpio_pin_config_t in_config = {
        kGPIO_DigitalInput,
        0
    };
    GPIO_PinInit(STOP_PORT, STOP_PIN, &in_config);
    GPIO_PinInit(GAS_PORT, GAS_PIN, &in_config);
    GPIO_PinInit(DHT_PORT, DHT_PIN, &in_config);

    // Enable Input Buffer (IBE) + Pull-Up (PE & PS) on digital input pins
    STOP_PORT_PERIPH->PCR[STOP_PIN] = PORT_PCR_MUX(0) | MCX_IBE_ENABLE | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    GAS_PORT_PERIPH->PCR[GAS_PIN]   = PORT_PCR_MUX(0) | MCX_IBE_ENABLE | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    DHT_PORT_PERIPH->PCR[DHT_PIN]   = PORT_PCR_MUX(0) | MCX_IBE_ENABLE | PORT_PCR_PE(1) | PORT_PCR_PS(1);

    PRINTF("\r\n=========================================\r\n");
    PRINTF("FreeRTOS Safety Monitor starting...\r\n");
    PRINTF("=========================================\r\n");

    // 3. Create FreeRTOS Queue & Mutex
    sensorQueue = xQueueCreate(4, sizeof(struct SensorData));
    stateMutex  = xSemaphoreCreateMutex();

    if (sensorQueue == NULL || stateMutex == NULL) {
        PRINTF("ERROR: Queue or Mutex creation failed!\r\n");
        while (1);
    }

    sharedState.gasDetected = false;
    sharedState.temperature_x10 = 290;
    sharedState.humidity_x10 = 700;
    sharedState.systemState = NORMAL;

    // 4. Create Tasks
    xTaskCreate(GasTask,         "Gas",    256, NULL, 3, NULL);
    xTaskCreate(EnvironmentTask, "Env",    256, NULL, 2, NULL);
    xTaskCreate(SafetyTask,      "Safety", 256, NULL, 4, NULL);
    xTaskCreate(LoggerTask,      "Logger", 256, NULL, 1, NULL);

    PRINTF("RTOS objects created. Starting Scheduler...\r\n");

    vTaskStartScheduler();

    while (1) {
    }

    return 0;
}