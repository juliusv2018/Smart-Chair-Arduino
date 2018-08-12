/*
 * Created in 2018/1/10
 * Modified in 2018/1/15 23:38
 * Implement the BLE processing
 * Author - Butterfly
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// For the BLE
#define THERMOMETER         BLEUUID((uint16_t)0x1809)
#define BATTERY_SERVICE     BLEUUID((uint16_t)0x180F)
#define DEVICEINFO          BLEUUID((uint16_t)0x180A)
#define CUSTOM              BLEUUID((uint16_t)0x1898)
#define HISTORY             BLEUUID((uint16_t)0x1899)

uint8_t batteryLevel = 90;
uint8_t temp[2] = {0b00000110, 25};

std::string manufacturerName = "Manager";
std::string modelNumber = "567-7985-57657-1309";
std::string serialNumber = "SMCH-10352ASCM";
std::string hardwareRevision = "Hardware Revision - Good";
std::string softwareRevision = "Software Revision - Good";

// [7:6]-2(Good Level), [5:4]-2(Not Charging, Chargable), [3:2]-3(Discharging), [1:0]-3(Present)
uint8_t batteryPowerState = 0b10101111;
uint32_t gyroActiveSeconds;
uint16_t chargingCycles;
QuaternionBLE quaternion;
HistoryInfo historyInfo;
RequestHistory requestHistory;
FetchHistory fetchHistory;
CrashInfo crashInfo;
RequestCrash requestCrash;
FetchCrash fetchCrash;

bool _BLEClientConnected = false;

BLECharacteristic batteryLevelCharacteristics(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic temperatureMeasurementCharacteristics(BLEUUID((uint16_t)0x2A1C), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic manufacturerNameCharacteristics(BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic modelNumberCharacteristics(BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
BLECharacteristic serialNumberCharacteristics(BLEUUID((uint16_t)0x2A25), BLECharacteristic::PROPERTY_READ);
BLECharacteristic hardwareRevisionCharacteristics(BLEUUID((uint16_t)0x2A27), BLECharacteristic::PROPERTY_READ);
BLECharacteristic softwareRevisionCharacteristics(BLEUUID((uint16_t)0x2A28), BLECharacteristic::PROPERTY_READ);
BLECharacteristic batteryPowerStateCharacteristics(BLEUUID((uint16_t)0x2A1A), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic gyroActiveCharacteristics(BLEUUID((uint16_t)0x2710), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic chargingCycleCharacteristics(BLEUUID((uint16_t)0x2711), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic quaternionCharacteristics(BLEUUID((uint16_t)0x2712), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//BLECharacteristic historyInfoCharacteristics(BLEUUID((uint16_t)0x2713), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//BLECharacteristic requestHistoryCharacteristics(BLEUUID((uint16_t)0x2714), BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic fetchHistoryCharacteristics(BLEUUID((uint16_t)0x2715), BLECharacteristic::PROPERTY_READ);
BLECharacteristic crashInfoCharacteristics(BLEUUID((uint16_t)0x2716), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic requestCrashCharacteristics(BLEUUID((uint16_t)0x2717), BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic fetchCrashCharacteristics(BLEUUID((uint16_t)0x2718), BLECharacteristic::PROPERTY_READ);

BLEDescriptor batteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor measurementDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor manufacturerNameDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor modelNumberDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor serialNumberDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor hardwareRevisionDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor softwareRevisionDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor batteryPowerStateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor gyroActiveDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor chargingCycleDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor quaternionDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor historyInfoDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor requestHistoryDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor fetchHistoryDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor crashInfoDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor requestCrashDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor fetchCrashDescriptor(BLEUUID((uint16_t)0x2901));

class ESP32BLEServer : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

class HistoryCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("History --> Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();
        Serial.println("*********");
      }

      // Do stuff based on the command received from the app
      // For some reason using rxValue.compare("A") == 0 doesn't work. Maybe
      // there are hidden characters I'm not seeing?
      if (rxValue.find("A") != -1) {
        Serial.println("Turning ON!");
      }
      else if (rxValue.find("B") != -1) {
        Serial.println("Turning OFF!");
      }
    }
};

class CrashCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Crash --> Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();
        Serial.println("*********");
      }

      // Do stuff based on the command received from the app
      // For some reason using rxValue.compare("A") == 0 doesn't work. Maybe
      // there are hidden characters I'm not seeing?
      if (rxValue.find("A") != -1) {
        Serial.println("Turning ON!");
      }
      else if (rxValue.find("B") != -1) {
        Serial.println("Turning OFF!");
      }
    }
};

void InitBLE()
{
  BLEDevice::init("Smart Chair BLE");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ESP32BLEServer());

  // Create the BLE battery service, Thermomter service
  BLEService *pBattery = pServer->createService(BATTERY_SERVICE);
  BLEService *pTemp = pServer->createService(THERMOMETER);
  BLEService *pDevInfo = pServer->createService(DEVICEINFO);
  BLEService *pCustom = pServer->createService(CUSTOM);
  BLEService *pHistory = pServer->createService(HISTORY);

  pBattery->addCharacteristic(&batteryLevelCharacteristics);
  batteryLevelDescriptor.setValue("Rate from 0 to 100");
  batteryLevelCharacteristics.addDescriptor(&batteryLevelDescriptor);
  batteryLevelCharacteristics.addDescriptor(new BLE2902());

  pTemp->addCharacteristic(&temperatureMeasurementCharacteristics);
  measurementDescriptor.setValue("Temperature value");
  temperatureMeasurementCharacteristics.addDescriptor(&measurementDescriptor);
  temperatureMeasurementCharacteristics.addDescriptor(new BLE2902());

  pDevInfo->addCharacteristic(&manufacturerNameCharacteristics);
  manufacturerNameDescriptor.setValue("Manufacturere name");
  manufacturerNameCharacteristics.addDescriptor(&manufacturerNameDescriptor);
  manufacturerNameCharacteristics.addDescriptor(new BLE2902());

  pDevInfo->addCharacteristic(&modelNumberCharacteristics);
  modelNumberDescriptor.setValue("Model number");
  modelNumberCharacteristics.addDescriptor(&modelNumberDescriptor);

  pDevInfo->addCharacteristic(&serialNumberCharacteristics);
  serialNumberDescriptor.setValue("Serial number");
  serialNumberCharacteristics.addDescriptor(&serialNumberDescriptor);

  pDevInfo->addCharacteristic(&hardwareRevisionCharacteristics);
  hardwareRevisionDescriptor.setValue("Hardware revision");
  hardwareRevisionCharacteristics.addDescriptor(&hardwareRevisionDescriptor);

  pDevInfo->addCharacteristic(&softwareRevisionCharacteristics);
  softwareRevisionDescriptor.setValue("Software revision");
  softwareRevisionCharacteristics.addDescriptor(&softwareRevisionDescriptor);

  // Battery power state
  pCustom->addCharacteristic(&batteryPowerStateCharacteristics);
  batteryPowerStateDescriptor.setValue("Battery Power State");
  batteryPowerStateCharacteristics.addDescriptor(&batteryPowerStateDescriptor);
  batteryPowerStateCharacteristics.addDescriptor(new BLE2902());

  // Gyro active seconds
  pCustom->addCharacteristic(&gyroActiveCharacteristics);
  gyroActiveDescriptor.setValue("Gyro active seconds");
  gyroActiveCharacteristics.addDescriptor(&gyroActiveDescriptor);
  gyroActiveCharacteristics.addDescriptor(new BLE2902());

  // Charging cycles
  pCustom->addCharacteristic(&chargingCycleCharacteristics);
  chargingCycleDescriptor.setValue("Charging cycles");
  chargingCycleCharacteristics.addDescriptor(&chargingCycleDescriptor);
  chargingCycleCharacteristics.addDescriptor(new BLE2902());

  // Quaternion
  pCustom->addCharacteristic(&quaternionCharacteristics);
  quaternionDescriptor.setValue("Current quaternion");
  quaternionCharacteristics.addDescriptor(&quaternionDescriptor);
  quaternionCharacteristics.addDescriptor(new BLE2902());
/*
  // History information
  pHistory->addCharacteristic(&historyInfoCharacteristics);
  historyInfoDescriptor.setValue("Information about the history storage");
  historyInfoCharacteristics.addDescriptor(&historyInfoDescriptor);
  historyInfoCharacteristics.addDescriptor(new BLE2902());

  // Request history
  pHistory->addCharacteristic(&requestHistoryCharacteristics);
  requestHistoryDescriptor.setValue("Request history for a specific day");
  requestHistoryCharacteristics.addDescriptor(&requestHistoryDescriptor);
  requestHistoryCharacteristics.setCallbacks(new HistoryCallback());
*/
  // Fetch history
  pHistory->addCharacteristic(&fetchHistoryCharacteristics);
  fetchHistoryDescriptor.setValue("Read history data");
  fetchHistoryCharacteristics.addDescriptor(&fetchHistoryDescriptor);

  // Crash information
  pHistory->addCharacteristic(&crashInfoCharacteristics);
  crashInfoDescriptor.setValue("Information about the crash history storage");
  crashInfoCharacteristics.addDescriptor(&crashInfoDescriptor);
  crashInfoCharacteristics.addDescriptor(new BLE2902());
/*
  // Request crash
  pHistory->addCharacteristic(&requestCrashCharacteristics);
  requestCrashDescriptor.setValue("Request crash data for a specific day");
  requestCrashCharacteristics.addDescriptor(&requestCrashDescriptor);
  requestCrashCharacteristics.setCallbacks(new CrashCallback());
*/
  // Fetch crash
  pHistory->addCharacteristic(&fetchCrashCharacteristics);
  fetchCrashDescriptor.setValue("Read crash data");
  fetchCrashCharacteristics.addDescriptor(&fetchCrashDescriptor);

  // Advertising BLE server
  pServer->getAdvertising()->addServiceUUID(BATTERY_SERVICE);
  pServer->getAdvertising()->addServiceUUID(THERMOMETER);
  pServer->getAdvertising()->addServiceUUID(DEVICEINFO);
  pServer->getAdvertising()->addServiceUUID(CUSTOM);
  pServer->getAdvertising()->addServiceUUID(HISTORY);

  pBattery->start();
  pTemp->start();
  pDevInfo->start();
  pCustom->start();
  pHistory->start();

  // Start advertising
  pServer->getAdvertising()->start();
}

void BLEProcessing()
{
  batteryLevelCharacteristics.setValue(&batteryLevel, 1);
  batteryLevelCharacteristics.notify();

  temperatureMeasurementCharacteristics.setValue(temp, 2);
  temperatureMeasurementCharacteristics.notify();

  manufacturerNameCharacteristics.setValue(manufacturerName);
  manufacturerNameCharacteristics.notify();

  modelNumberCharacteristics.setValue(modelNumber);
  serialNumberCharacteristics.setValue(serialNumber);
  hardwareRevisionCharacteristics.setValue(hardwareRevision);
  softwareRevisionCharacteristics.setValue(softwareRevision);

  batteryPowerStateCharacteristics.setValue(&batteryPowerState, sizeof(batteryPowerState));
  batteryPowerStateCharacteristics.notify();

  gyroActiveCharacteristics.setValue((uint8_t *)&gyroActiveSeconds, sizeof(gyroActiveSeconds));
  gyroActiveCharacteristics.notify();

  chargingCycleCharacteristics.setValue((uint8_t *)&chargingCycles, sizeof(chargingCycles));
  chargingCycleCharacteristics.notify();

  quaternionCharacteristics.setValue((uint8_t *)&quaternion, sizeof(quaternion));
  quaternionCharacteristics.notify();
/*
  historyInfoCharacteristics.setValue((uint8_t *)&historyInfo, sizeof(historyInfo));
  historyInfoCharacteristics.notify();
*/
  fetchHistoryCharacteristics.setValue((uint8_t *)&fetchHistory, sizeof(fetchHistory));
  crashInfoCharacteristics.setValue((uint8_t *)&crashInfo, sizeof(crashInfo));
  crashInfoCharacteristics.notify();

  fetchCrashCharacteristics.setValue((uint8_t *)&fetchCrash, sizeof(fetchCrash));
}

