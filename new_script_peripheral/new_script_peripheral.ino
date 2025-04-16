#include <bluefruit.h>

// UUID definitions (must match your Python script)
#define SENSOR_SERVICE_UUID "290bd33f-8e7e-485b-b18a-6f8d28482d75"
#define AD_SERVICE_UUID "390bd33f-8e7e-485b-b18a-6f8d28482d75"

#define NOTIFY_CHAR_UUID "290bd33f-8e7e-485b-b18a-6f8d28482d76"
#define S_WRITE_CHAR_UUID "290bd33f-8e7e-485b-b18a-6f8d28482d77"
#define AD_WRITE_CHAR_UUID "390bd33f-8e7e-485b-b18a-6f8d28482d78"

// Create service objects
BLEService sensorService = BLEService(SENSOR_SERVICE_UUID);
BLEService adService = BLEService(AD_SERVICE_UUID);

// Create characteristic objects
BLECharacteristic notifyChar = BLECharacteristic(NOTIFY_CHAR_UUID);
BLECharacteristic sWriteChar = BLECharacteristic(S_WRITE_CHAR_UUID);
BLECharacteristic adWriteChar = BLECharacteristic(AD_WRITE_CHAR_UUID);

// Callback to handle writes to the advertising characteristic
void adWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 4) {  // 2 bytes each = 4 bytes total
    uint16_t ad_time = ((uint16_t*)data)[0];
    uint16_t sleep_time = ((uint16_t*)data)[1];

    Serial.print("Received Advertising Time (ms): ");
    Serial.println(ad_time);
    Serial.print("Received Sleep Time (s): ");
    Serial.println(sleep_time);
  } else {
    Serial.println("Invalid data length received. Expected 8 bytes.");
  }
}

void sWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 5) {  // Expecting 1 byte + 2 bytes + 2 bytes = 5 bytes
    uint8_t sensorSelection = data[0];
    uint16_t odr = *((uint16_t*)(data + 1));
    uint16_t duration = *((uint16_t*)(data + 3));

    Serial.print("Received Sensor Selection: ");
    Serial.println(sensorSelection);
    Serial.print("Received ODR: ");
    Serial.println(odr);
    Serial.print("Received Duration: ");
    Serial.println(duration);

    // Add your sensor setup logic here based on received values
  }
  else if (len == 1) {
    // This handles the earlier initial 1-byte write for just sensor selection
    uint8_t startSampling = data[0];
    Serial.print("Received command to start sampling: ");
    Serial.println(startSampling);
  } 
  else {
    Serial.print("Invalid S_WRITE_CHAR data length received. Expected 1 or 5 bytes, got: ");
    Serial.println(len);
  }
}


void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);

  Bluefruit.begin();
  Bluefruit.setName("AAX");

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  // Start services
  sensorService.begin();

  // Configure characteristics
  notifyChar.setProperties(CHR_PROPS_NOTIFY);
  notifyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // sensorService.addCharacteristic(notifyChar);
  notifyChar.begin();

  sWriteChar.setProperties(CHR_PROPS_WRITE);
  sWriteChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  sWriteChar.setWriteCallback(sWriteCallback);
  // sensorService.addCharacteristic(sWriteChar);
  sWriteChar.begin();


  adService.begin();

  adWriteChar.setProperties(CHR_PROPS_WRITE);
  adWriteChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  adWriteChar.setWriteCallback(adWriteCallback);  // Set the callback function
  // adService.addCharacteristic(adWriteChar);      // IMPORTANT: Add the characteristic to the service
  adWriteChar.begin();

  // Advertise both services
  Bluefruit.Advertising.addService(sensorService);
  // Bluefruit.Advertising.addService(adService);  // <- Make sure this is added

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(244, 244);
  Bluefruit.Advertising.start(0);

  Serial.println("Advertising as AAX...");
}

void loop() {
  if (Bluefruit.connected()) {
    Serial.println("Connected");
    delay(1000);
  }
}