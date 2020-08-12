#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "AudioAnalyzer.h"

/**
  << This Device Master >>
  // Sample Serial log with 1 master & 1 slave
     Found 12 devices
     1: Slave:3C:71:BF:52:D1:14 [3C:71:BF:52:D1:15] (-42)
     1 Slave(s) found, processing..
     Processing: 3C:71:BF:52:D1:15 Status: Pair success
     Digital Read: 1
     Sending: 1
     Send Status: Success
     Last Packet Sent to: 3c:71:bf:52:d1:15
     Last Packet Send Status: Delivery Success
*/

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 3
#define NUM_SLAVES 2       // ESP-Now can handle a maximum of 20 slaves, im here using only a max of 2 slaves
#define PRINTSCANRESULTS 0

int slaveCount = 0;                     // Keeps count of no. of slaves with the defined prefix
esp_now_peer_info_t slaves[NUM_SLAVES]; // Stores the information of each of the slave that is added as a peer

void initESPNow();
void manageSlaves();
void scanForSlaves();
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_add, const uint8_t *data, int data_len);
void sendData(uint8_t data);


////
/* define event group and event bits */
EventGroupHandle_t eg;
#define evtDo_AudioReadFreq       ( 1 << 0 ) // 1
////
QueueHandle_t xQ_LED_Info;
////
const int freq =0;
////
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.

////
void setup()
{
   Serial.begin(115200);
  eg = xEventGroupCreate();
  //  Audio.Init(); // start the audio analyzer
    ////
  int FreqVal[7];
  xQ_LED_Info = xQueueCreate ( 1, sizeof(FreqVal) );
  //////////////////////////////////////////////////////////////////////////////////////////////
  xTaskCreatePinnedToCore( fDo_AudioReadFreq, "fDo_ AudioReadFreq", 40000, NULL, 4, NULL, 1 ); //assigned to core
  xTaskCreatePinnedToCore( fDo_SendData, "ffDo_ SendData", 40000, NULL, 4, NULL, 0 ); //assigned to core
  xEventGroupSetBits( eg, evtDo_AudioReadFreq );

pinMode(LED_BUILTIN, OUTPUT);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_MODE_STA);

  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());

  // Init ESPNow with a fallback logic
  initESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  scanForSlaves();
  manageSlaves();
  
} // setup()
////
void loop() {vTaskDelete;} // void loop
////
void fDo_SendData( void *pvParameters )
{
  int iFreqVal[7];  
  for (;;)
  {
    if (xQueueReceive( xQ_LED_Info, &iFreqVal,  portMAX_DELAY) == pdTRUE)
    {
    sendData( iFreqVal[freq]);
    }
    xEventGroupSetBits( eg, evtDo_AudioReadFreq );
  }
  vTaskDelete( NULL );
} // void fDo_ LEDs( void *pvParameters )
////
void fDo_AudioReadFreq( void *pvParameters )
{
  int FreqVal[7];
  const int NOISE = 10; // noise that you want to chop off
  const int A_D_ConversionBits = 4096; // arduino use 1024, ESP32 use 4096
  Analyzer Audio = Analyzer(26,27,35);//Strobe pin ->26  RST pin ->27 Analog Pin ->35
  Audio.Init(); // start the audio analyzer
  int64_t EndTime = esp_timer_get_time();
  int64_t StartTime = esp_timer_get_time(); //gets time in uSeconds like Arduino Micros
  for (;;)
  {
    xEventGroupWaitBits (eg, evtDo_AudioReadFreq, pdTRUE, pdTRUE, portMAX_DELAY);
    EndTime = esp_timer_get_time() - StartTime;
     log_i( "TimeSpentOnTasks: %d", EndTime );
    Audio.ReadFreq(FreqVal);
    for (int i = 0; i < 7; i++)
    {
      FreqVal[i] = constrain( FreqVal[i], NOISE, A_D_ConversionBits );
      FreqVal[i] = map( FreqVal[i], NOISE, A_D_ConversionBits, 0, 250 );
       log_i( "Freq %d Value: %d", i, FreqVal[i]);//used for debugging and Freq choosing
    }
    xQueueSend( xQ_LED_Info, ( void * ) &FreqVal, 0 );
    StartTime = esp_timer_get_time();
  }
  vTaskDelete( NULL );
} // fDo_ AudioReadFreq( void *pvParameters )
////
// Init ESP Now with fallback
void initESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void scanForSlaves()
{
  int8_t scanResults = WiFi.scanNetworks();

  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  slaveCount = 0;
  Serial.println("");

  if (scanResults == 0)
  {
    Serial.println("No WiFi devices in AP Mode found");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");

    for (int i = 0; i < scanResults; i++)
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS)
      {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }

      delay(10);

      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0)
      {
        // SSID of interest
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");

        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int j = 0; j < 6; j++)
          {
            slaves[slaveCount].peer_addr[j] = (uint8_t)mac[j];
          }
        }

        slaves[slaveCount].channel = CHANNEL; // pick a channel
        slaves[slaveCount].encrypt = 0; // no encryption
        slaveCount++;
      }
    }
  }

  if (slaveCount > 0)
  {
    Serial.print(slaveCount); Serial.println(" Slave(s) found, processing..");
  }
  else
  {
    Serial.println("No Slave Found, trying again.");
  }
delay(5000);
  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlaves()
{
  if (slaveCount > 0)
  {
    for (int i = 0; i < slaveCount; i++)
    {
      const esp_now_peer_info_t *peer = &slaves[i];
      const uint8_t *peer_addr = slaves[i].peer_addr;
      Serial.print("Processing: ");

      for (int j = 0; j < 6; j++)
      {
        Serial.print((uint8_t) slaves[i].peer_addr[j], HEX);
        if (j != 5)
        {
          Serial.print(":");
        }
      }

      Serial.print(" Status: ");

      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peer_addr);
      if (exists)
      {
        // Slave already paired.
        Serial.println("Already Paired");
      }
      else
      {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK)
        {
          // Pair success
          Serial.println("Pair success");
        }
        else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
        {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        }
        else if (addStatus == ESP_ERR_ESPNOW_ARG)
        {
          Serial.println("Add Peer - Invalid Argument");
        }
        else if (addStatus == ESP_ERR_ESPNOW_FULL)
        {
          Serial.println("Peer list full");
        }
        else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
        {
          Serial.println("Out of memory");
        }
        else if (addStatus == ESP_ERR_ESPNOW_EXIST)
        {
          Serial.println("Peer Exists");
        }
        else
        {
          Serial.println("Not sure what happened");
        }
        delay(5000);
      }
    }
  }
  else
  {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


// send data
void sendData(uint8_t data) {
  
  for (int i = 0; i < slaveCount; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
   if (i == 0) { // print only for first slave
      Serial.print("Sending: ");
      Serial.println(data);
    }
   esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
    Serial.print("Send Status: ");

    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(10);
  }
}

// callback when data is sent from Master to Slave
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{

  char macStr[18];
  //sendTime = millis();
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  //Serial.print("Last Packet Sent to: ");
 // Serial.println(macStr);
 Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");


}

// callback when data is received from Slave to Master
void onDataRecv(const uint8_t *mac_add, const uint8_t *data, int data_len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_add[0], mac_add[1], mac_add[2], mac_add[3], mac_add[4], mac_add[5]);

  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(*data);
  Serial.println("");
}
