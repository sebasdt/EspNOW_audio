
#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"




#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

////
/* define event group and event bits */
EventGroupHandle_t eg;
////
QueueHandle_t queueReceved;
////
const int LED_COUNT = 50; //total number of leds in the strip

////
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.

////
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  eg = xEventGroupCreate();
  //  leds.begin(); // Call this to start up the LED strip.
  //  clearLEDs( LED_COUNT );  // This function, defined below, de-energizes all LEDs...
  //  leds.show();  // ...but the LEDs don't actually update until you call this.
  ////
  queueReceved = xQueueCreate ( 1, sizeof(int));
  //////////////////////////////////////////////////////////////////////////////////////////////
  xTaskCreatePinnedToCore( fDo_LEDs, "fDo_ LEDs", 40000, NULL, 4, NULL, 0 ); //assigned to core
} // setup()
////
void loop() {
  vTaskDelete(NULL); // void loop
}
////
void fDo_LEDs( void *pvParameters )
{
  int FreqVal;

  for (;;)
  {
    if (xQueueReceive( queueReceved, &FreqVal,  portMAX_DELAY) == pdTRUE)
    {
      Serial.println(FreqVal);

    }



  }
  vTaskDelete( NULL );
} // void fDo_ LEDs( void *pvParameters )
////
//the following function set the led color based on its position and freq value
//

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ///Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  //  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  // Serial.println("");
  int rFreqVal = *data;
  xQueueSend( queueReceved,&rFreqVal , portMAX_DELAY);
  //Serial.println(data);
}
