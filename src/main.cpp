#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "I2Cdev.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLE2902.h"
#include "MPU6050_6Axis_MotionApps20.h"
// pre-determined UUID so windows recognize nunchuck as BLE HID
#define SERVICE_UUID  "1812"
#define INFO_UUID     "2A4A"
#define MAP_UUID      "2A4B"
#define CONTROL_UUID  "2A4C"
#define REPORT_UUID   "2A4D"
#define PROTOCOL_UUID "2A4E"
#define OUTPUT_READABLE_QUATERNION
#define JOY_DEADZONE 150  // deadzone to mitigate noise when joystick is still



//===============================================================================================================
// current issue : joystick axis reading reach max value even when the stick hasnt reach its mechanical limit
// another issue : all characteristic values are copy pasted from AI (make sure to learn and modify it)



// MPU6050 Related variables
MPU6050 mpuSensor;
Quaternion quaternionData;                   // contain quaternion value [w, x, y, z]
uint8_t FIFO_buffer[64];
TaskHandle_t MPU_readHandle = NULL;
int const INTERRUPT_PIN = 34;
int const SDA_PIN = 21;
int const SCL_PIN = 22;
float xAxis, yAxis, zAxis, wValue;            // variables to store each axis value from quaternionData


// MPU6050 Related Functions
void MPU_activate()  // initiate MPU6050 and its DMP
{
  Wire.begin(SDA_PIN, SCL_PIN);     // activate I2C communication
  pinMode(INTERRUPT_PIN, INPUT);
  while (true)
  {
    mpuSensor.initialize();
    mpuSensor.dmpInitialize();
    if (mpuSensor.testConnection())
    {
      Serial.println("MPU6050 Connected");
      return;
    }
    Serial.println("MPU6050 Connection Failed. Retrying...");
    vTaskDelay(pdTICKS_TO_MS(500));
  }
}

void MPU_calibrate() // do initial calibration for MPU6050
{
  Serial.println("Calibrating Sensor.....");
  mpuSensor.CalibrateAccel(10);           // sampling 10 times to calibrate acceleration sensor
  mpuSensor.CalibrateGyro(10);            // sampling 10 times to calibrate gyro sensor
  mpuSensor.setDMPEnabled(true);          // activate DMP chip using this config
  Serial.println("Sensor Successfully Calibrated!");
  return;
}

void MPU_read(void* params)  // read quaternion value from MPU6050
{
  while (true)
  {
    if (mpuSensor.dmpGetCurrentFIFOPacket(FIFO_buffer))
    {
      mpuSensor.dmpGetQuaternion(&quaternionData, FIFO_buffer);
      xAxis = quaternionData.x;  yAxis = quaternionData.y;  zAxis = quaternionData.z;  wValue = quaternionData.w;
      //Serial.println(String("Sensor Read : ") + xAxis + " , " + yAxis + " , " + zAxis + " , " + wValue);
    }
    vTaskDelay(4);    // MPU6050 send packet every ~10 ms, so make sure the program capture it at 2x the rate (nyquist theorem) (change this line later to 4 ms)
  }
}



//===============================================================================================================



// Joystick Related Variables
TaskHandle_t JOY_triggerHandle = NULL;
TaskHandle_t JOY_getAxisHandle = NULL;
int const JOY_BUTTON_PIN = 12;  // use pin with internal pullup so we dont use external for pullup and caused temporary floating
int const JOY_X_PIN = 13;
int const JOY_Y_PIN = 14;
volatile uint32_t lastPress = 0;
int centerX, centerY;


// Joystick Related Functions
void IRAM_ATTR JOY_ISR()
{
  uint32_t now = millis();
  if (now - lastPress > 100)
  {
    lastPress = now;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(JOY_triggerHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void JOY_calibrate()
{
  int samplingX = 0;
  int samplingY = 0;
  Serial.println("Calibrating Joystick.....");
  for (int i = 0; i < 50; i++)
  {
    samplingX += analogRead(JOY_X_PIN);
    samplingY += analogRead(JOY_Y_PIN);
  }
  centerX = samplingX / 50;
  centerY = samplingY / 50;
  Serial.println("Joystick Successfully Calibrated!");
}

int JOY_readAxis(int const JOY_AXIS_PIN)
{
  int sum = 0;
  for (int i = 0; i < 8; i++)
  {
    sum += analogRead(JOY_AXIS_PIN);
  }
  return sum / 8;
}

void JOY_trigger(void* params)
{
  while (true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("Joystick Pressed!");
  }
}

void JOY_getAxis(void* params)
{
  int axisValue[2];
  while(true)
  {
    axisValue[0] = JOY_readAxis(JOY_X_PIN);
    axisValue[1] = JOY_readAxis(JOY_Y_PIN);
    // if the joystick position is inside deadzone, ignore the value
    if (abs(axisValue[0] - centerX) < JOY_DEADZONE) axisValue[0] = 0; 
    if (abs(axisValue[1] - centerY) < JOY_DEADZONE) axisValue[1] = 0;
    //Serial.println("axis : " + String(axisValue[0]) + "," + String(axisValue[1]));
    vTaskDelay(10);   // 10 - 20 ms usually a sweet spot for standard BLE joystick (console can go faster)
  }
}

// note: joystick button default at high and set to low when pressed, so use pin with internal pullup to match the logic



//===============================================================================================================



// TTP233 Related Variables
TaskHandle_t TPP_triggerHandle = NULL;
int const TTP233_PIN = 15;
volatile uint32_t lastTouch = 0;


// TTP233 Related Functions
void IRAM_ATTR TPP_ISR()  // send notification to TPP_trigger when interrupt occur
{
  uint32_t now = millis();
  if (now - lastTouch > 100)
  {
    lastTouch = now;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(TPP_triggerHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void TPP_trigger(void* params) // handle logic based on interrupt event
{
  while (true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("Trigger Button is Pressed!");
  }
}
// note : TPP sensor is low by default and going high when touched



//===============================================================================================================



// BLE Related Variables
BLEServer *nunchuckServer;
BLEService *nunchuckService;
BLECharacteristic *informationCharacteristic;
BLECharacteristic *reportMapCharacteristic;
BLECharacteristic *controlPointCharacteristic;
BLECharacteristic *reportCharacteristic;
BLECharacteristic *protocolModeCharacteristic;
BLEAdvertising *nunchuckAdvertising;
BLEDescriptor *reportRef;


// BLE Related Functions
void InitBLE()
{
  Serial.println("Configuring BLE.....");
  BLEDevice::init("Custom Nunchuck");
  nunchuckServer = BLEDevice::createServer();
  nunchuckService = nunchuckServer->createService(SERVICE_UUID);

  // general information (one per service)
  informationCharacteristic = nunchuckService->createCharacteristic(INFO_UUID, BLECharacteristic::PROPERTY_READ);
  uint8_t infoValue[4] = {
    0x11, 0x01, // HID version 1.11 (format in little endian so the subversion number is upfront)
    0x00,       // Country code none (nunchuck dont need localization)
    0x02        // Flags in normal connection mode (maintain connection and cant turn on connected device remotely)
  };
  informationCharacteristic->setValue(infoValue, 4);

  // report map (one per service, can be connected to multiple report characteristics)
  reportMapCharacteristic = nunchuckService->createCharacteristic(MAP_UUID, BLECharacteristic::PROPERTY_READ);
  uint8_t hidReportMap[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        // Report ID (anything below this will belong to report with ID 1 until another report ID is declared)
    // add more here according to need
    0xC0               // End Collection
  };
  reportMapCharacteristic->setValue(hidReportMap, sizeof(hidReportMap));

  // control point
  controlPointCharacteristic = nunchuckService->createCharacteristic(CONTROL_UUID, BLECharacteristic::PROPERTY_WRITE_NR);

  // report characteristic (can be many, each have its own ID)
  reportCharacteristic = nunchuckService->createCharacteristic(REPORT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  reportCharacteristic->addDescriptor(new BLE2902()); // Client Characteristic Configuration Descriptor (control notification)
  reportRef = new BLEDescriptor("2908");  // difine metadata for this report (ID, type, etc..)
  uint8_t reportRefValue[2] = {
    0x01, // Report ID
    0x01  // Report type as input for connected device (0x02 -> output, 0x03 -> two-way)
  };
  reportRef->setValue(reportRefValue, 2);
  reportCharacteristic->addDescriptor(reportRef);

  // protocol mode
  protocolModeCharacteristic = nunchuckService->createCharacteristic(PROTOCOL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
  uint8_t protocolMode = 0x01;
  protocolModeCharacteristic->setValue(&protocolMode, 1);
  nunchuckService->start();

  nunchuckAdvertising = BLEDevice::getAdvertising();
  nunchuckAdvertising->addServiceUUID(SERVICE_UUID);
  nunchuckAdvertising->setScanResponse(true);         // send more detailed metadata so client dont display unknown device
  BLEDevice::startAdvertising();
  Serial.println("Configuration Done! BLE is ready to use");
  return;
}



//===============================================================================================================



void setup() 
{
  Serial.begin(115200);            
  vTaskDelay(pdTICKS_TO_MS(1000));
  pinMode(JOY_BUTTON_PIN, INPUT_PULLUP);

  MPU_activate();
  MPU_calibrate();
  JOY_calibrate();

  xTaskCreatePinnedToCore(  // task to handle data reading from MPU6050
    MPU_read,
    "ReadMPUTask",
    4096,
    NULL,
    1,
    &MPU_readHandle,
    0
  );
  xTaskCreatePinnedToCore(
    JOY_getAxis,
    "JoystickGetAxisTask",
    2048,
    NULL,
    1,
    &JOY_getAxisHandle,
    0
  );
  xTaskCreatePinnedToCore(  // task to handle TPP touch sensor trigger
    TPP_trigger, 
    "TouchSensorTask", 
    1024, 
    NULL, 
    1, 
    &TPP_triggerHandle, 
    1
  ); 
  xTaskCreatePinnedToCore(
    JOY_trigger,
    "JoystickTriggerTask",
    1024,
    NULL,
    1,
    &JOY_triggerHandle,
    1
  );

  attachInterrupt(digitalPinToInterrupt(TTP233_PIN), &TPP_ISR, RISING);   // TPP ISR should be called when D15 going HIGH
  attachInterrupt(digitalPinToInterrupt(JOY_BUTTON_PIN), &JOY_ISR, FALLING);   // joystick ISR chould be called when D18 going HIGH
  InitBLE();  // configure Bluetooth Low Energy (BLE)
}

void loop() 
{

}
