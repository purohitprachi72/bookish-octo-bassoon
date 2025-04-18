#include <bluefruit.h>
#include <SPI.h>
#include <SparkFun_KX13X.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#include <PDM.h>

Adafruit_FlashTransport_QSPI flashTransport;

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

TaskHandle_t TASK_H = NULL;

enum SelectSensor {
  NONE = 0x00,
  KX134_ = 0x01,
  IMU_ = 0x02,
  PDM_ = 0x03
};

volatile uint16_t odr_value = 0;
volatile uint16_t duration = 0;
volatile SelectSensor init_sensor = NONE;
volatile bool start_sampling = false;

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // Global IMU object
uint16_t sampleRate = 1660;

SparkFun_KX134_SPI kxAccel;  // Global KX object
rawOutputData myData;
const byte chipSelect = 1;

// PDM specific variables - IMPROVED PDM SECTION
short pdmSampleBuffer[512];  // Buffer size for PDM data
volatile int pdmSamplesRead = 0;
volatile bool pdmSamplingActive = false;
volatile uint32_t pdmSamplesCollected = 0;
volatile uint32_t pdmTotalSamplesToCollect = 0;

// Maximum buffer size (adjust according to your RAM)
#define MAX_BUFFER_SIZE 200000
uint8_t sensorBuffer[MAX_BUFFER_SIZE];
volatile uint32_t bufferIndex = 0;
volatile bool bufferReady = false;

// BLE packet size (MTU minus overhead)
#define BLE_PACKET_SIZE 240  // Based on MTU of 247, minus overhead
volatile bool transmitting = false;

// Function pointer types for initialization and sampling
typedef void (*InitFunctionPtr)();
typedef void (*SampleFunctionPtr)(uint16_t odr, uint16_t duration);

// Function pointers to hold the right functions for the selected sensor
InitFunctionPtr initFunction = NULL;
SampleFunctionPtr sampleFunction = NULL;

// Forward declarations for sensor functions
void initKX134();
void sampleKX134(uint16_t odr, uint16_t duration);
void initIMU();
void sampleIMU(uint16_t odr, uint16_t duration);
void initPDM();
void samplePDM(uint16_t odr, uint16_t duration);
void sendBufferOverBLE();
void checkPDMStatus();

// Calculate time between samples based on ODR
uint32_t calculateMicrosPerSample(uint16_t odr) {
  return 1000000 / odr;  // Microseconds per sample
}

void adWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 4) {  // 2 bytes each = 4 bytes total
    uint16_t ad_time = ((uint16_t*)data)[0];
    uint16_t sleep_time = ((uint16_t*)data)[1];

    Serial.print("Received Advertising Time (ms): ");
    Serial.println(ad_time);
    Serial.print("Received Sleep Time (s): ");
    Serial.println(sleep_time);
  } else {
    Serial.println("Invalid data length received. Expected 4 bytes.");
  }
}

void sWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 5) {
    init_sensor = static_cast<SelectSensor>(data[0]);
    odr_value = *((uint16_t*)(data + 1));
    duration = *((uint16_t*)(data + 3));

    Serial.print("Selected sensor: ");
    Serial.println(init_sensor);
    Serial.print("ODR: ");
    Serial.println(odr_value);
    Serial.print("Duration (s): ");
    Serial.println(duration);

    // Reset buffer
    bufferIndex = 0;
    bufferReady = false;
    transmitting = false;

    switch (init_sensor) {
      case KX134_:
        initFunction = initKX134;
        sampleFunction = sampleKX134;
        break;
      case IMU_:
        initFunction = initIMU;
        sampleFunction = sampleIMU;
        break;
      case PDM_:
        initFunction = initPDM;
        sampleFunction = samplePDM;
        break;
      default:
        initFunction = NULL;
        sampleFunction = NULL;
        break;
    }
  } else if (len == 1 && data[0] == 0x01) {
    start_sampling = true;
    vTaskResume(TASK_H);
  } else {
    Serial.print("Invalid data length received. Expected 1 or 5 bytes, got: ");
    Serial.println(len);
  }
}

void vTask(void* p) {
  for (;;) {
    // If a new sensor was selected, initialize it
    if (initFunction != NULL) {
      initFunction();
      Serial.print("Sensor ");
      Serial.print(init_sensor);
      Serial.println(" initialized!");

      // Clear the initFunction to indicate we've handled the initialization
      initFunction = NULL;
    }

    // If sampling was requested and we have a valid sampling function
    if (start_sampling && sampleFunction != NULL) {
      Serial.println("Starting sampling...");
      sampleFunction(odr_value, duration);
      start_sampling = false;  // Reset the flag

      // After sampling is complete, send the buffer over BLE
      if (bufferReady) {
        Serial.println("Sampling complete. Starting transmission...");
        transmitting = true;
        sendBufferOverBLE();
      }
    }

    vTaskSuspend(TASK_H);
  }
}

void sendBufferOverBLE() {
  uint32_t totalBytes = bufferIndex;
  uint32_t bytesSent = 0;
  uint16_t packetSize = BLE_PACKET_SIZE;

  Serial.print("Total bytes to send: ");
  Serial.println(totalBytes);

  while (bytesSent < totalBytes) {
    // Calculate the size of the current packet
    uint16_t currentPacketSize = min(packetSize, totalBytes - bytesSent);

    // Send a chunk of data
    if (Bluefruit.connected() && notifyChar.notify(&sensorBuffer[bytesSent], currentPacketSize)) {
      bytesSent += currentPacketSize;

      // Print progress every 10 packets
      if ((bytesSent / packetSize) % 10 == 0) {
        Serial.print("Sent ");
        Serial.print(bytesSent);
        Serial.print(" of ");
        Serial.print(totalBytes);
        Serial.println(" bytes");
      }

      // Small delay to avoid overwhelming the BLE stack
      delay(5);
    } else {
      Serial.println("BLE notify failed or disconnected");
      delay(100);  // Wait longer if there's an issue

      // If disconnected, stop transmission
      if (!Bluefruit.connected()) {
        Serial.println("BLE disconnected during transmission");
        break;
      }
    }
  }

  Serial.println("Buffer transmission complete");
  transmitting = false;
  bufferReady = false;
}

void initKX134() {
  // Initialize KX134 sensor
  Serial.println("Initializing KX134...");

  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  SPI.begin();
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
  if (!kxAccel.begin(SPI, settings, chipSelect)) {
    Serial.println("Could not communicate with the KX13X sensor. Halting.");
    while (1)
      ;
  }

  if (kxAccel.softwareReset()) {
    Serial.println("Sensor reset successful.");
  }
  delay(5);

  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX134_RANGE64G);
  kxAccel.enableDataEngine();
}

void sampleKX134(uint16_t odr, uint16_t duration) {

  if (odr == 5000) {
    kxAccel.setOutputDataRate(13);
  } else {
    kxAccel.setOutputDataRate(14);
  }

  kxAccel.enableAccel();

  Serial.print("Sampling KX134 with ODR: ");
  Serial.print(odr);
  Serial.print(" for duration: ");
  Serial.println(duration);

  // Reset buffer index
  bufferIndex = 0;

  // Calculate total samples based on ODR and duration
  uint32_t totalSamples = odr * duration;
  uint32_t bytesPerSample = 3 * sizeof(int16_t);  // 3 axes, each 2 bytes
  uint32_t totalBytes = totalSamples * bytesPerSample;

  // Check if buffer is large enough
  if (totalBytes > MAX_BUFFER_SIZE) {
    Serial.println("ERROR: Buffer too small for requested duration");
    totalBytes = MAX_BUFFER_SIZE;
    Serial.println("Setting totalBytes = MAX_BUFFER_SIZE (200000)");
    // return;
  }

  // Calculate microseconds per sample
  uint32_t microsPerSample = calculateMicrosPerSample(odr);
  uint32_t nextSampleTime = micros();

  // Sample for the requested duration
  for (uint32_t i = 0; i < totalSamples; i++) {
    // Wait until it's time for the next sample
    while (micros() < nextSampleTime) {
      // Busy wait or yield to other tasks
      yield();
    }

    // Get new sample time
    nextSampleTime += microsPerSample;

    // Read KX134 data
    kxAccel.getRawAccelData(&myData);

    // Store data
    int16_t x = myData.xData;
    int16_t y = myData.yData;
    int16_t z = myData.zData;

    // Store values in buffer (each axis is 2 bytes)
    memcpy(&sensorBuffer[bufferIndex], &x, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &y, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &z, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
  }

  kxAccel.enableAccel(false);

  Serial.print("Collected ");
  Serial.print(bufferIndex);
  Serial.println(" bytes of data");

  bufferReady = true;
}

void initIMU() {
  // Initialize IMU sensor
  Serial.println("Initializing IMU...");

  myIMU.settings.accelSampleRate = sampleRate;
  myIMU.settings.gyroSampleRate = sampleRate;
  myIMU.settings.accelBandWidth = 16;
  myIMU.settings.gyroBandWidth = 500;
  myIMU.settings.tempEnabled = 0;

  delay(50);

  if (myIMU.begin() != 0) {
    Serial.println("IMU setup complete");
  } else {
    Serial.println("IMU setup failed!");
    // return;
  }

  Wire1.setClock(400000);  // For faster I2C read

  // Enable Low Power modes
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, LSM6DS3_ACC_GYRO_LP_XL_ENABLED);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, LSM6DS3_ACC_GYRO_LP_EN_ENABLED);
}

void sampleIMU(uint16_t odr, uint16_t duration) {
  Serial.print("Sampling IMU with ODR: ");
  Serial.print(odr);
  Serial.print(" for duration: ");
  Serial.println(duration);

  // Reset buffer index
  bufferIndex = 0;

  // Calculate total samples based on ODR and duration
  uint32_t totalSamples = odr * duration;
  uint32_t bytesPerSample = 6 * sizeof(int16_t);  // 6 axes, each 2 bytes
  uint32_t totalBytes = totalSamples * bytesPerSample;

  // Check if buffer is large enough
  if (totalBytes > MAX_BUFFER_SIZE) {
    Serial.println("ERROR: Buffer too small for requested duration");
    totalBytes = MAX_BUFFER_SIZE;
    Serial.println("Setting totalBytes = MAX_BUFFER_SIZE (200000)");
    // return;
  }

  // Calculate microseconds per sample
  uint32_t microsPerSample = calculateMicrosPerSample(odr);
  uint32_t nextSampleTime = micros();

  // Sample for the requested duration
  for (uint32_t i = 0; i < totalSamples; i++) {
    // Wait until it's time for the next sample
    while (micros() < nextSampleTime) {
      // Busy wait or yield to other tasks
      yield();
    }

    // Get new sample time
    nextSampleTime += microsPerSample;

    // Read IMU data
    int16_t ax = myIMU.readRawAccelX();
    int16_t ay = myIMU.readRawAccelY();
    int16_t az = myIMU.readRawAccelZ();
    int16_t gx = myIMU.readRawGyroX();
    int16_t gy = myIMU.readRawGyroY();
    int16_t gz = myIMU.readRawGyroZ();

    // Store values in buffer (each axis is 2 bytes)
    memcpy(&sensorBuffer[bufferIndex], &ax, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &ay, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &az, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &gx, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &gy, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
    memcpy(&sensorBuffer[bufferIndex], &gz, sizeof(int16_t));
    bufferIndex += sizeof(int16_t);
  }

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, LSM6DS3_ACC_GYRO_BW_XL_400Hz | LSM6DS3_ACC_GYRO_FS_XL_16g | LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN);

  Serial.print("Collected ");
  Serial.print(bufferIndex);
  Serial.println(" bytes of data");

  bufferReady = true;
}

// IMPROVED PDM FUNCTIONS WITH FIXES

// PDM callback function - executed when PDM data is available
void onPDMdata() {
  // Get the number of bytes available
  int bytesAvailable = PDM.available();
  
  if (bytesAvailable <= 0) return;
  
  // Read into the sample buffer
  int bytesRead = PDM.read(pdmSampleBuffer, bytesAvailable);
  
  // Only process data if we're actively sampling
  if (!pdmSamplingActive) return;
  
  // Calculate number of samples (16-bit, 2 bytes per sample)
  int samplesRead = bytesRead / 2;
  
  // Debug output for first few samples
  if (pdmSamplesCollected < 10) {
    Serial.print("PDM received ");
    Serial.print(bytesRead);
    Serial.print(" bytes (");
    Serial.print(samplesRead);
    Serial.println(" samples)");
  }
  
  // Copy data to our main sensor buffer
  if (samplesRead > 0 && (pdmSamplesCollected < pdmTotalSamplesToCollect)) {
    // Don't exceed our target sample count
    int samplesToStore = min(samplesRead, pdmTotalSamplesToCollect - pdmSamplesCollected);
    
    // Check if we have enough space in the buffer
    if (bufferIndex + (samplesToStore * sizeof(int16_t)) <= MAX_BUFFER_SIZE) {
      // Copy samples to main buffer
      memcpy(&sensorBuffer[bufferIndex], pdmSampleBuffer, samplesToStore * sizeof(int16_t));
      bufferIndex += samplesToStore * sizeof(int16_t);
      pdmSamplesCollected += samplesToStore;
      
      // Check if we've collected all samples
      if (pdmSamplesCollected >= pdmTotalSamplesToCollect) {
        pdmSamplingActive = false;
        bufferReady = true;
        Serial.print("PDM collection complete. Collected ");
        Serial.print(pdmSamplesCollected);
        Serial.println(" samples.");
      }
    } else {
      // Buffer is full
      pdmSamplingActive = false;
      bufferReady = true;
      Serial.println("PDM buffer is full");
    }
  }
}

void initPDM() {
  // Initialize PDM sensor
  Serial.println("Initializing PDM...");

  // End any previous PDM sessions
  PDM.end();
  delay(100);  // Give time for cleanup
  
  // Configure the PDM module
  PDM.onReceive(onPDMdata);
  
  PDM.setGain(40);  // Increase gain (0-80)
  PDM.setBufferSize(512);  // Keep buffer size
  
  // Start PDM with MONO channel at 16KHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM! Retrying...");
    delay(1000);
    if (!PDM.begin(1, 16000)) {  // Try again
      Serial.println("PDM initialization failed again!");
    }
  } else {
    Serial.println("PDM initialized successfully");
  }

  // Reset variables
  pdmSamplesRead = 0;
  pdmSamplingActive = false;
  pdmSamplesCollected = 0;
}

void samplePDM(uint16_t odr, uint16_t duration) {
  Serial.print("Sampling PDM with sample rate: ");
  Serial.print(odr);
  Serial.print(" Hz for duration: ");
  Serial.print(duration);
  Serial.println(" seconds");

  // Reset buffer index and counters
  bufferIndex = 0;
  pdmSamplesCollected = 0;
  
  // For PDM, the ODR is our actual audio sample rate (typically 16000 Hz)
  // Calculate total samples based on ODR and duration
  pdmTotalSamplesToCollect = odr * duration;
  uint32_t totalBytes = pdmTotalSamplesToCollect * sizeof(int16_t);

  // Check if buffer is large enough
  if (totalBytes > MAX_BUFFER_SIZE) {
    Serial.println("WARNING: Buffer too small for requested duration");
    Serial.print("Can store max ");
    Serial.print(MAX_BUFFER_SIZE / sizeof(int16_t));
    Serial.println(" samples");
    pdmTotalSamplesToCollect = MAX_BUFFER_SIZE / sizeof(int16_t);
  }
  
  Serial.print("Will collect ");
  Serial.print(pdmTotalSamplesToCollect);
  Serial.println(" audio samples");

  // Force restart PDM to ensure it's active
  PDM.end();
  delay(100);
  
  // Set callback again after PDM.end()
  PDM.onReceive(onPDMdata);
  PDM.setGain(40);
  PDM.setBufferSize(512);
  
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to restart PDM!");
    return;
  }
  
  Serial.println("PDM successfully restarted for sampling");

  // Start PDM sampling
  pdmSamplingActive = true;

  // Wait for sampling to complete
  uint32_t startTime = millis();
  uint32_t timeout = duration * 1000 + 2000;  // Duration in ms plus 2 second buffer
  uint32_t lastOutputTime = 0;
  uint32_t lastDebugTime = 0;

  while (pdmSamplingActive && (millis() - startTime < timeout)) {
    // Monitor progress every second
    if (millis() - lastOutputTime >= 1000) {
      lastOutputTime = millis();
      Serial.print("PDM progress: ");
      Serial.print(pdmSamplesCollected);
      Serial.print(" of ");
      Serial.print(pdmTotalSamplesToCollect);
      Serial.print(" samples (");
      Serial.print((pdmSamplesCollected * 100) / max(pdmTotalSamplesToCollect, 1)); // Avoid div by zero
      Serial.println("%)");
    }
    
    // If no samples after 2 seconds, force a debug check
    if (pdmSamplesCollected == 0 && (millis() - lastDebugTime >= 2000)) {
      lastDebugTime = millis();
      Serial.println("PDM Debug: No samples received yet. Checking status...");
      
      // Additional debug to check if callback is working
      int bytesAvailable = PDM.available();
      Serial.print("PDM.available() returns: ");
      Serial.println(bytesAvailable);
      
      // Try reading data directly to see if PDM is working
      if (bytesAvailable > 0) {
        short tempBuffer[128];
        int bytesRead = PDM.read(tempBuffer, bytesAvailable);
        Serial.print("Direct PDM read: ");
        Serial.print(bytesRead);
        Serial.println(" bytes");
        
        // Display first few values if available
        if (bytesRead > 0) {
          Serial.print("First values: ");
          for (int i = 0; i < min(5, bytesRead/2); i++) {
            Serial.print(tempBuffer[i]);
            Serial.print(" ");
          }
          Serial.println();
        }
      }
    }
    
    yield();  // Allow other tasks to run
  }

  // Check if we timed out
  if (pdmSamplingActive) {
    Serial.println("PDM sampling timed out!");
    pdmSamplingActive = false;
    
    // Even if we timed out, mark what we have as ready
    if (pdmSamplesCollected > 0) {
      bufferReady = true;
    }
  }

  // Stop PDM
  PDM.end();

  Serial.print("PDM sampling complete. Collected ");
  Serial.print(bufferIndex);
  Serial.print(" bytes (");
  Serial.print(pdmSamplesCollected);
  Serial.println(" samples)");
}

// Debug function for PDM status
void checkPDMStatus() {
  Serial.println("\n--- PDM Status ---");
  Serial.print("PDM active: ");
  Serial.println(pdmSamplingActive ? "Yes" : "No");
  Serial.print("Samples collected: ");
  Serial.println(pdmSamplesCollected);
  Serial.print("Target samples: ");
  Serial.println(pdmTotalSamplesToCollect);
  Serial.print("Buffer index: ");
  Serial.println(bufferIndex);
  Serial.print("Buffer ready: ");
  Serial.println(bufferReady ? "Yes" : "No");
  
  // Try to read PDM directly
  int bytesAvailable = PDM.available();
  Serial.print("PDM.available() returns: ");
  Serial.println(bytesAvailable);
  Serial.println("-----------------\n");
}

void setup() {
  // Enable DC-DC converter
  NRF_POWER->DCDCEN = 1;  // Enable DC/DC converter for REG1 stage

  // Flash power-down mode
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // enter deep power-down mode
  delayMicroseconds(5);
  flashTransport.end();

  Serial.begin(115200);
  Serial.flush();
  // while (!Serial) delay(10);  // Uncomment for debugging

  Serial.println("BLE Sensor Controller");

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName("AAX");
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);

  // Set up sensor service
  sensorService.begin();

  // Notify characteristic for sending sensor data
  notifyChar.setProperties(CHR_PROPS_NOTIFY);
  notifyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  notifyChar.setMaxLen(BLE_PACKET_SIZE);
  notifyChar.begin();

  // Write characteristic for sensor control
  sWriteChar.setProperties(CHR_PROPS_WRITE);
  sWriteChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  sWriteChar.setWriteCallback(sWriteCallback);
  sWriteChar.begin();

  // Set up advertising service
  adService.begin();
  adWriteChar.setProperties(CHR_PROPS_WRITE);
  adWriteChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  adWriteChar.setWriteCallback(adWriteCallback);
  adWriteChar.begin();

  // Setup advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);  // Advertise the sensor service
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(244, 244);
  Bluefruit.Advertising.setFastTimeout(1);
  Bluefruit.Advertising.start(0);

  Serial.println("Started advertising...");
  Serial.flush();

  // Create the task
  xTaskCreate(vTask,
              "Task",
              configMINIMAL_STACK_SIZE * 4,  // Increased stack size for safety
              NULL,
              1,
              &TASK_H);

  vTaskSuspend(TASK_H);

  suspendLoop();
}

void loop() {
  // Loop is suspended
}

void connect_cb(uint16_t conn_handle) {
  Serial.println("Connected");
  
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(6, 1, 10000);  // ~7.5-10 ms connection interval
  vTaskResume(TASK_H);
}

void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);

  // If we were transmitting data, reset the flags
  if (transmitting) {
    transmitting = false;
    bufferReady = false;
  }
  
  // Turn off PDM if it was active
  if (pdmSamplingActive) {
    pdmSamplingActive = false;
    PDM.end();
  }
}
