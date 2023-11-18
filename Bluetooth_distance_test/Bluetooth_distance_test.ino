#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertisedDevice.h>
#include "kalman_simple.h"
 
// Comment this line out for the final version (terse output in the serial monitor)
#define VERBOSE

BLECharacteristic *pCharacteristic;
 
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
 
#define SERVICE_UUID        "6012d087-05f8-41b0-90ed-07a75e80a104"
#define CHARACTERISTIC_UUID "73af693f-f43f-4997-817c-3c226530ad76"
#define DESCRIPTOR_UUID    "e8f45c7e-8be5-4918-bb84-467d3fd354aa"

double measurement, filteredMeasurement;
Kalman myFilter(0.08, 50, 50, -50.);
// 0.08, 50 gave decent, if somewhat slow, results)
 
template <typename T, size_t N>
void show_address(const T (&address)[N])
{
  Serial.print(address[0], HEX);
  for (uint8_t i = 1; i < N; i++)
  {
    Serial.printf(":%02x", address[i]);
  }
}
 
 
class Monitor: public BLEServerCallbacks
{
public:
  static int16_t connection_id;

  Monitor() //: connection_id(-1), remote_addr({})
  {
    
  }

  /* dBm to distance parameters; How to update distance_factor 
   *  1. place the phone at a known distance (2m, 3m, 5m, 10m) 
   *  2. average about 10 RSSI values for each of these distances 
   *  3. Set distance_factor so that the calculated distance approaches the actual distances, e.g. at 5m. */
  static constexpr float reference_power  = -54; // rssi reference (Power at 1 meter)
  static constexpr float distance_factor = 3.5; 

  
  esp_err_t getRSSI() 
  { 
    return esp_ble_gap_read_rssi(remote_addr); 
  }
 
  static float getDistance(const float rssi)
  { 
    return pow(10, (reference_power - rssi) / (10 * distance_factor)); 
  }
 
private:
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) 
  {
    // Update connection variables
    connection_id = param->connect.conn_id;
    memcpy(&remote_addr, param->connect.remote_bda, sizeof(remote_addr));
    
    // Install the RSSI callback
    BLEDevice::setCustomGapHandler(&Monitor::rssi_event);

  #ifdef VERBOSE
    // Show new connection info
    Serial.printf("Connection #: %i, remote: ", connection_id);
    show_address(param->connect.remote_bda);
    Serial.printf(" [Callback installed]\n");
  #endif
  }
 
  void onDisconnect(BLEServer* pServer)
  {
    Serial.printf("Connection #%i closed\n", connection_id);
    BLEDevice::setCustomGapHandler(nullptr);
    connection_id = -1;
  }
 
  static void rssi_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
  {
    static double rssi_average = 0.0;
   
  #ifdef VERBOSE
    //show_address(remote_addr);
  #endif
    if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT)
    {
      Serial.println((double)param->read_rssi_cmpl.rssi);
      rssi_average = (double)myFilter.getFilteredValue((double)param->read_rssi_cmpl.rssi);
  
    #ifdef VERBOSE
      Serial.printf(", rssi=%hi, rssi_average =%g,  distance~=%g \n",param->read_rssi_cmpl.rssi, rssi_average, getDistance(rssi_average));
    #else
      Serial.printf("%hi, %g\n", param->read_rssi_cmpl.rssi, getDistance(rssi_average));
    #endif
      
    }
  }
 
  static esp_bd_addr_t remote_addr;
};
 
int16_t Monitor::connection_id = -1;
esp_bd_addr_t Monitor::remote_addr = {};
 

 
Monitor monitor;
 
void setup()
{
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Esp-32");
 
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(&monitor);
 
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
 
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
 
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLEDescriptor(DESCRIPTOR_UUID));
 
  // Start the service
  pService->start();
 
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for incoming connections...");
}
 
void loop()
{
  static const uint32_t REFRESH_DELAY = 150;
  static uint32_t next_detection;

  uint32_t current_time = millis();
  if (Monitor::connection_id != -1)
  {
    if (current_time - next_detection >= REFRESH_DELAY)
    {
      // Prepare for the next detection
      next_detection += REFRESH_DELAY;

      // Update the internal value (what for?)
      //auto value = monitor.get_value();
      //Serial.printf("*** NOTIFY: %d ***\n", value);
      //pCharacteristic->setValue(&value, sizeof(value));
      //pCharacteristic->notify();
   
      // Request RSSI from the remote address
      if (monitor.getRSSI() != ESP_OK)
      {
        Serial.println("RSSI request failed");
      }
    }
  }
}
