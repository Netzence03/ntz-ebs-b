#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <ModbusMaster.h>
#include <LiquidCrystal.h>
#include "utils.h"
#include "cellular.h"

/*
Define network of choice using the following macro:
#define AIRTEL_NETWORK
#define MTN_NETWORK
#define GLO_NETWORK
#define 9MOBILE_NETWORK
*/
#define WDT_TIMEOUT 3000
#define AIRTEL_NETWORK
#include <network_config.h>

#define DEBUG true
#define UART_BAUD 115200

#define MODEM_TX 16
#define MODEM_RX 17
#define MODEM_PWRKEY 4
#define MODEM_DTR 5

#define MODBUS_TX 27
#define MODBUS_RX 25
#define MODBUS_DIR 26
#define MODBUS_BAUD 9600

#define MODBUS_SLAVE_ADDR 7
#define VOLUME_DATA_SIZE 2
#define VOLUME_INT_REGISTER 42

#define DEVICE_ID "NGF-NGS-3072565"
#define TOKEN "1269090669f2f3759bdb94c18fbb5726a31bbe4b13b8a2453e380752334d1a47"
#define FACILITY_TYPE "non-green"
#define FUEL_TYPE "natural-gas"
#define VVALUE 4.2999
#define ENDPOINT "https://73291s35t8.execute-api.us-east-1.amazonaws.com/dev-stage/submit-emission-data"
#define BASE_CALC_APPROACH "fuel-consumed"

#define DATA_INTERVAL 30
#define COMM_RESUME_TIME 5
#define HOUR 3600

// Define LCD pins
#define LCD_RS 19
#define LCD_EN 21
#define LCD_D4 15
#define LCD_D5 16
#define LCD_D6 17
#define LCD_D7 18

// Define LCD dimensions
#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define MEMORY_DEBUG false
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define Terminal Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1
#define MODBUS_SERIAL Serial2
int a = esp_random();
Preferences preferences;
SemaphoreHandle_t preferenceMutex;

uint32_t volumeValue = 0;
uint32_t oldVolumeValue = 0;
uint32_t newVolumeValue = 0;
uint32_t intervalValue = 0;
uint32_t volume;
bool send_data = false;
TaskHandle_t modbusTaskHandle;
TaskHandle_t commTaskHandle;

String http_status = "";

// Initialize the ModbusMaster object as modbus
ModbusMaster modbus;

// MODBUS_DIR made high for Modbus transmision mode
void modbusPreTransmission()
{
  vTaskDelay(500 / portTICK_PERIOD_MS);
  digitalWrite(MODBUS_DIR, HIGH);
}
// MODBUS_DIR made low for Modbus receive mode
void modbusPostTransmission()
{
  digitalWrite(MODBUS_DIR, LOW);
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void comm_task(void *pvParameters)
{

  while (1)
  {
    Terminal.println("COMM STACK: " + (String)uxTaskGetStackHighWaterMark(NULL));
    if (xSemaphoreTake(preferenceMutex, portMAX_DELAY) == pdTRUE)
    {
      Terminal.println("Semaphore Taken by Comm1");
      preferences.begin("storage", false);
      intervalValue = preferences.getUInt("interval", 0);
      oldVolumeValue = preferences.getUInt("old_volume", 0);
      newVolumeValue = preferences.getUInt("new_volume", 0);
      preferences.end();
      xSemaphoreGive(preferenceMutex);
    }

    if (send_data)
    {
      Terminal.println("Start Sending...");
      esp_task_wdt_reset();
      uint32_t volume_m3 = newVolumeValue - oldVolumeValue;
      String http_str = "AT+HTTPPARA=\"URL\",\"" + (String)ENDPOINT + "\"\r\n";

      String data = "{\"deviceId\" : \"" + (String)DEVICE_ID + "\", \"value\" : \"" + (String)volume_m3 + "\", \"token\" : \"" + (String)TOKEN + "\", \"facilityType\" : \"" + (String)FACILITY_TYPE + "\", \"vValue\" : \"" + (String)VVALUE + "\", \"key\" : \"2\", \"baseCalcApraoch\" : \"" + (String)BASE_CALC_APPROACH + "\"}";

      // String data = "{\"deviceId\" : \"" + (String)DEVICE_ID + "\", \"value\" : \"" + "3456" + "\", \"token\" : \"" + (String)TOKEN + "\", \"facilityType\" : \"" + (String)FACILITY_TYPE + "\", \"vValue\" : \"" + (String)VVALUE + "\"}";

      Terminal.println(http_str);

      sendData("AT+HTTPINIT\r\n", 2000, DEBUG);
      sendData(http_str, 2000, DEBUG);
      sendData("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", 2000, DEBUG);
      sendData("AT+HTTPDATA=" + (String)data.length() + ",10000\r\n", 2000, DEBUG);
      sendData(data + "\r\n", 3000, DEBUG);
      http_status = sendData("AT+HTTPACTION=1\r\n", 3000, DEBUG); // This will hold the server status
      sendData("AT+HTTPTERM\r\n", 3000, DEBUG);

      send_data = false;

      // Put communication module to sleep
      // sendData("AT+CSCLK=1", 1000, DEBUG);
      digitalWrite(MODEM_DTR, HIGH);

      // Suspend communication task
      vTaskSuspend(NULL);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void modbus_task(void *pvParameters)
{
  while (1)
  {
    long start_time = millis();
    esp_task_wdt_reset();
    Terminal.println("MODBUS STACK: " + (String)uxTaskGetStackHighWaterMark(NULL));

    if (intervalValue == 0)
    {
      uint8_t j, result;
      uint16_t buf[VOLUME_DATA_SIZE];
      // uint16_t temp;

      Terminal.println("Reading registers");
      result = modbus.readHoldingRegisters(VOLUME_INT_REGISTER, VOLUME_DATA_SIZE);

      if (result == modbus.ku8MBSuccess)
      {
        Terminal.println("Success! Processing...");
        for (j = 0; j < VOLUME_DATA_SIZE; j++)
        {
          buf[j] = modbus.getResponseBuffer(j);
          Terminal.print(buf[j]);
          Terminal.print(" ");
        }
        Terminal.println("<- done");
        // swap bytes because the data comes in Big Endian!
        int buf_size = sizeof(buf) / sizeof(buf[0]);
        reverseBuffer(buf, buf_size);
        // hand-assemble a single-precision uint32_t from the bytestream
        memcpy(&volume, &buf, sizeof(uint32_t));
        Terminal.print("Volume is ");
        Terminal.println(volume);
      }
      else
      {
        Terminal.print("Failure. Code: ");
        Terminal.println(result);
      }
    }
    intervalValue++;
    if (xSemaphoreTake(preferenceMutex, portMAX_DELAY) == pdTRUE)
    {
      Terminal.println("Semaphore Taken by Modbus");
      preferences.begin("storage", false);

      // Read data from flash
      // volumeValue = preferences.getLong64("volume", volumeValue);

      // volumeValue = volumeValue + volume;

      preferences.putUInt("interval", intervalValue);
      intervalValue = preferences.getUInt("interval", intervalValue);

      if (intervalValue >= (DATA_INTERVAL - COMM_RESUME_TIME))
      {
        // Wake up communication module
        digitalWrite(MODEM_DTR, LOW);

        // Resume communication task
        vTaskResume(commTaskHandle);
      }

      if (intervalValue == DATA_INTERVAL)
      {
        send_data = true;
        preferences.putUInt("interval", 0);
        intervalValue = preferences.getUInt("interval", intervalValue);
      }

      if (intervalValue == 0)
      {
        Terminal.println("INTERVAL IS ZERO !");
        oldVolumeValue = preferences.getUInt("new_volume", 0);
        if (oldVolumeValue == 0)
        {
          preferences.putUInt("new_volume", volume);
          preferences.putUInt("old_volume", volume);
        }
        else
        {
          preferences.putUInt("old_volume", oldVolumeValue);
          preferences.putUInt("new_volume", volume);
        }
      }

      Terminal.println("Current Interval: " + (String)intervalValue);

      preferences.end();
      xSemaphoreGive(preferenceMutex);
    }
    long process_time = millis() - start_time;
    long delay_s = (process_time > 1000) ? 0 : (1000 - process_time);
    vTaskDelay(delay_s / portTICK_PERIOD_MS);
    Terminal.println("PROCESS TIME: " + (String)process_time);
  }
}

void setup()
{
  esp_task_wdt_init(WDT_TIMEOUT, DEBUG); // Set watchdog timeout
  esp_task_wdt_add(NULL);

#if MEMORY_DEBUG
  Terminal.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Terminal.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Terminal.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Terminal.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
#endif
  // Set console baud rate
  Terminal.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Set 4G module baud rate
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);
  /*
  MODEM_PWRKEY IO:4 The power-on signal of the modulator must be given to it,
  otherwise the modulator will not reply when the command is sent
  */ 
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  vTaskDelay(300 / portTICK_PERIOD_MS); // Need delay
  digitalWrite(MODEM_PWRKEY, LOW);

  pinMode(MODBUS_DIR, OUTPUT);
  digitalWrite(MODBUS_DIR, LOW);
  MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX, MODBUS_TX);
  MODBUS_SERIAL.setTimeout(2000);
  modbus.begin(MODBUS_SLAVE_ADDR, MODBUS_SERIAL);

  //  Register post- and pre- transmission Callbacks
  modbus.preTransmission(modbusPreTransmission);
  modbus.postTransmission(modbusPostTransmission);

#if MEMORY_DEBUG
  Terminal.println((String)uxTaskGetStackHighWaterMark(NULL));
#endif
  sendData("AT+CSCLK=1", 1000, DEBUG); // UART can sleep when DTR pull high
  sendData("AT+CCID", 3000, DEBUG); 
  sendData("AT+CREG?", 3000, DEBUG);
  sendData("AT+CGATT=1", 1000, DEBUG); // Attach packet domain to enable data service (internet access) and other IP-based communications
  sendData("AT+CGACT=1,1", 1000, DEBUG);
  sendData("AT+CGDCONT=1,\"IP\",\"" + (String)APN + "\"", 1000, DEBUG);
  // String COMMAND = "AT+CGAUTH=1,1,\"" + (String)NETWORK_USER + "\",\"" + (String)NETWORK_PWD + "\"";
  // sendData("AT+CGAUTH=1,1,\"" + (String)NETWORK_USER + "\",\"" + (String)NETWORK_PWD + "\"", 2000, DEBUG);
  // Serial.println(COMMAND);
  Serial.println("4G HTTP Test Begin!");

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  preferenceMutex = xSemaphoreCreateMutex();

  esp_err_t err = nvs_flash_init();

  nvs_flash_erase();
  nvs_flash_init();

  // Set up the LCD's number of columns and rows:
  lcd.begin(LCD_COLS, LCD_ROWS);
  // Print a message to the LCD.
  lcd.print("ESP32 LCD Demo");

  // Create tasks to run on each core
  xTaskCreatePinnedToCore(
      modbus_task,       // Task function
      "modbus_task",     // Task name
      4000,             // Stack size
      NULL,              // Task parameters
      10,                // Priority
      &modbusTaskHandle, // Task handle
      1                  // Run on core 1
  );

  xTaskCreatePinnedToCore(
      comm_task,       // Task function
      "comm_task",     // Task name
      4000,           // Stack size
      NULL,            // Task parameters
      10,              // Priority
      &commTaskHandle, // Task handle
      0                // Run on core 0
  );
  // vTaskSuspend(commTaskHandle);
  // Delete setup task running on priority 1 on Core 1
  vTaskDelete(NULL);
}

void loop()
{
  // Main loop, but not used
  // Delete loop task running on priority 1 on Core 1
  vTaskDelete(NULL);
}
