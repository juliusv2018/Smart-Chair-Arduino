/*
   Modified in 2018/1/15 23:45
   Organize all functions and remove dmp service
   Author - Butterfly
*/

#include <MPU6050.h>
#include <esp32/ulp.h>
#include <RTClib.h>
#include <Preferences.h>

#include "define.h"
#include "function.h"
#include "bleprocess.h"
#include "header.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         MAIN PROGRAM                                                //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Take some time to open up the Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  // RTC Initialization
  Serial.println("RTC begin...");
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");

    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
  Serial.println("Device Initialization");  
  mpuInitialize();
  
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  printWakeupReason();

  deviceInitialize();
}

void loop()
{
  if (state == INITIAL_STATE)
  {
    Serial.println("\n-------- Initial State ---------\n");
    Serial.println("Starting the device...");

    // MPU6050 Initialization
    Serial.println("MPU6050 Initialization...");
    mpuInitialize();

    Serial.println("Low Power Mode Setting...");
    lowPowerModeSetting();

    // start sleep mode
    Serial.println("Start deep sleep mode...\n");
    esp_deep_sleep_start();
  }
  else if (state == LOW_BATTERY_STATE)
  {
    // Low battery, wake from the ulp source
    Serial.println("\n--------- Low Battery State ---------");
    Serial.println("Power off...");
    digitalWrite(LDOEnable, LOW);
  }
  else if (state == EXT1_AWAKE_STATE)
  {
    Serial.println("\n-------- Real Time State ---------");
    // Real Time Mode State
    Serial.println("Realtime state Settings...");
    realtimeModeSetting();
    Serial.println("BLE service Start...");
    InitBLE();

    // RTC Initialization
    Serial.println("RTC begin...");
    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      while (1);
    }

    // Start by performing self test and reporting values
    mpu6050SelfTest(SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
      calibrateMPU6050(flGyroBias, flAccelBias);

      // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      // initMPU6050();
      // Serial.println("MPU6050 initialized for active data mode....");
    }
    else
    {
      // Loop forever if communication doesn't happen
      Serial.print("Could not connect to MPU6050: 0x");
      while (1);
    }
    gyroActiveSecondsTime = rtc.now();
    bleOldTime = gyroActiveSecondsTime.second();

    // Varible Initialization
    variableInitialize();
    state = REALTIME_STATE;
  }
  else if (state == REALTIME_STATE)	// Real Time State
  {
    if (!rtc.isrunning()) {
      Serial.println("RTC is NOT running!");
    }

    if (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)	{
      // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
      // Read the x/y/z adc values
      readAccelData(accelCount);
      getAres();

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * aRes;
      ay = (float)accelCount[1] * aRes;
      az = (float)accelCount[2] * aRes;

      if (ax > 1.3 || ay > 1.3 || az > 1.5) {
        intOldTime = rtc.now();
      }

      // Read the x/y/z adc values
      readGyroData(gyroCount);
      getGres();

      // Calculate the gyro value into actual degrees per second
      gx = (float)gyroCount[0] * gRes;
      gy = (float)gyroCount[1] * gRes;
      gz = (float)gyroCount[2] * gRes;

      // Read the x/y/z adc values
      // Temperature in degrees Centigrade
      tempCount = readTempData();

      // Calculate Quaternion
      Now = micros();
      deltat = ((Now - lastUpdate) / 1000000.0f);		// set integration time by time elapsed since last filter update
      lastUpdate = Now;

      // Pass gyro rate as rad/s
      MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f);
      yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw *= 180.0f / PI;
      roll *= 180.0f / PI;

      // Calculate quaternion - for store
      quaternion.W = (int16_t)((q[0] + 1) * 10000);
      quaternion.X = (int16_t)((q[1] + 1) * 10000);
      quaternion.Y = (int16_t)((q[2] + 1) * 10000);
      quaternion.Z = (int16_t)((q[3] + 1) * 10000);
      /*
            Serial.print("Quaternion Test : ");
            Serial.print(q[0]);
            Serial.print(", ");
            Serial.print(q[1]);
            Serial.print(", ");
            Serial.print(q[2]);
            Serial.print(", ");
            Serial.println(q[3]);
      */
      decideBusyTime();
      decideCrashState();
    }

    // measure the time of no motion, if it exceeds to limit, the esp goes into sleep mode.
    DateTime datetime = rtc.now();

    // BLE processing part per second
    if (bleOldTime != datetime.second())
    {
      bleOldTime = datetime.second();

      // get the battery value
      uint16_t batteryADCValue = analogRead(ADCBattery);
      batteryLevel = (uint8_t) map(batteryADCValue, 780, 1024, 0, 100);

      // get the battery power state
      // battery good or critical level
      if (batteryLevel > 25) {                      // good level
        batteryPowerState &= 0b10111111;
        batteryPowerState |= 0b10000000;
      }
      else {                                        // critical level
        batteryPowerState |= 0b11000000;
      }

      // Charging or No Charging
      if (digitalRead(CHARGESTATE_PIN) == LOW) {    // charging
        batteryPowerState |= 0b00110000;
      }
      else {                                        // Not Charging
        batteryPowerState &= 0b11101111;
        batteryPowerState |= 0b00100000;
      }

      // Get temperature
      temp[1] = (uint8_t)(tempCount / 340.0 + 36.53);
      gyroActiveSeconds = (uint32_t)(datetime.secondstime() - gyroActiveSecondsTime.secondstime());

      preferences.getBytes("HistoryEntryTest", (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
      fetchHistory.data = (uint8_t *)&saveHistoryEntry;

      preferences.getBytes("CrashTest", (void *)&crashData, sizeof(crashData));
      fetchCrash.data = (uint8_t *)&crashData;

      // Update BLE service data
      BLEProcessing();
      Serial.print(".");
    }

    if ((datetime.secondstime() - intOldTime.secondstime()) > NO_MOTION_SECONDS_TIME)
    {
      Serial.println("\n========================================================");
      Serial.println("-----------------End of Realtime state------------------");
      Serial.println("----------------No more motion detected-----------------");
      Serial.println("\n");
      Serial.println("Low Power Mode Setting...");
      lowPowerModeSetting();

      // start sleep mode
      Serial.println("Start deep sleep mode...");
      esp_deep_sleep_start();
    }
  }
  else if (state == SAVE_HISTORY_STATE)
  {
    DateTime datetime = rtc.now();

    preferences.getBytes("LastHistorySaveTime", (void *)&lastHistorySaveTime, sizeof(DateTime));
    if (lastHistorySaveTime.year() != datetime.year() || lastHistorySaveTime.month() != datetime.month() || lastHistorySaveTime.day() != datetime.day())
    {
      uint8_t currentSavePosition = preferences.getChar("HistorySavePosition", 0);
      saveHistoryEntry.date.day = datetime.day();
      saveHistoryEntry.date.month = datetime.month();
      saveHistoryEntry.date.year = datetime.year();

      currentSavePosition ++;
      if (currentSavePosition == CRASH_SAVE_LIMIT) {
        currentSavePosition = 0;
      }

      // Save the history
      char keySequence[15] = "";
      String("HistoryEntry" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
      preferences.putBytes(keySequence, (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
      // Save the last time
      preferences.putBytes("LastHistorySaveTime", (void *)&datetime, sizeof(DateTime));
    }

    esp_deep_sleep_start();
  }
}

// MPU Initialization
void mpuInitialize()
{
  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // TWBR = 24;		// 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize device
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  // All FIFO enabled except for temperature FIFO
  mpu.setTempFIFOEnabled(false);
  mpu.setXGyroFIFOEnabled(false);
  mpu.setYGyroFIFOEnabled(false);
  mpu.setZGyroFIFOEnabled(false);
  mpu.setAccelFIFOEnabled(false);
}

// Device Initialization
void deviceInitialize()
{
  // LDOEnable pin is set to 1 in the initial stage.
  pinMode(LDOEnable, OUTPUT);
  digitalWrite(LDOEnable, HIGH);

  pinMode(CHARGESTATE_PIN, INPUT);
  pinMode(CHARGELED_PIN, OUTPUT);
  digitalWrite(CHARGELED_PIN, ~(digitalRead(CHARGESTATE_PIN)));

  // MPU interrupt pin setting as MPU_INT_PIN
  pinMode(MPU_INT_PIN, INPUT);

  // ULP initialization
  ulpInitialize();

  // Wakeup sources setup
  esp_sleep_enable_timer_wakeup(300 * uS_TO_S_FACTOR);							            // to save history, 5 minutes timer wakeup
  esp_sleep_enable_ext1_wakeup((1 << MPU_INT_PIN), ESP_EXT1_WAKEUP_ANY_HIGH);		// 1 = High, 0 = Low, MPU_INT_PIN - GPIO25
  esp_sleep_enable_ulp_wakeup();
}

// Variables Initialization
void variableInitialize()
{
  nowBusy = false;
  intOldTime = rtc.now();
}

// Checking the wakeup reason
void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by EXT0"); break;		// Wakeup caused by Ext0
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by EXT1");
      state = EXT1_AWAKE_STATE; break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by Timer");
      state = SAVE_HISTORY_STATE; break;					      // Wakeup caused by 5 minutes timer
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP"); break;
      state = LOW_BATTERY_STATE; break;					        // Wakeup caused by ULP
    default:
      Serial.println("Wakeup was not caused by deep sleep");
      state = INITIAL_STATE; break;
  }
}

// Setting for Low power mode
void lowPowerModeSetting()
{
  // XYZ gyro in standby, wake frequency = 1 -> 0B00111000void setStandbyXGyroEnabled(bool enabled);
  // mpu.setStandbyXAccelEnabled(1);
  // mpu.setStandbyYAccelEnabled(1);
  // mpu.setStandbyZAccelEnabled(1);

  mpu.setStandbyXGyroEnabled(1);
  mpu.setStandbyYGyroEnabled(1);
  mpu.setStandbyZGyroEnabled(1);

  mpu.resetFIFO();
  mpu.resetSensors();

  mpu.setWakeCycleEnabled(0);
  mpu.setDHPFMode(0);
  mpu.setDLPFMode(0);

  // Free fall detection thr = 70, duration = 1, counter decrement = 1
  mpu.setFreefallDetectionThreshold(70);
  mpu.setFreefallDetectionDuration(1);
  mpu.setFreefallDetectionCounterDecrement(1);

  mpu.setMotionDetectionThreshold(20);
  mpu.setMotionDetectionDuration(1);
  mpu.setMotionDetectionCounterDecrement(1);

  mpu.setIntDataReadyEnabled(0);
  mpu.setIntFreefallEnabled(0);
  mpu.setIntMotionEnabled(1);
  mpu.setIntFIFOBufferOverflowEnabled(0);

  mpu.setDHPFMode(1);
  mpu.setWakeFrequency(1);
  mpu.setWakeCycleEnabled(1);
}

// Setting for Real time mode
void realtimeModeSetting()
{
  mpu.setWakeCycleEnabled(false);

  // XYZ gyro not in standby, wake frequency = 1 -> 0B00111111
  mpu.setStandbyXGyroEnabled(false);
  mpu.setStandbyYGyroEnabled(false);
  mpu.setStandbyZGyroEnabled(false);

  // kill motion detection interrupt
  mpu.setIntMotionEnabled(0);
  mpu.setIntDataReadyEnabled(1);
  mpu.setDLPFMode(3);

  // mpu.reset();
  // delay(100);
  // initMPU6050();

  intOldTime = rtc.now();
}

// Decide whether the chair is busy or not
void decideBusyTime()
{
  if ((ax > BUSY_TIME_VALUE_THR) || (ay > BUSY_TIME_VALUE_THR) || (az > BUSY_TIME_VALUE_THR))
  {
    if ((rtc.now().secondstime() - intOldTime.secondstime() > BUSY_TIME_DURATION_LTHR) && (rtc.now().secondstime() - intOldTime.secondstime() < BUSY_TIME_DURATION_HTHR))
    {
      DateTime datetime = rtc.now();

      // Save History, wake from the timer
      uint8_t currentSavePosition = preferences.getChar("HistorySavePosition", 0);
      preferences.getBytes("LastHistorySaveTime", (void *)&lastHistorySaveTime, sizeof(DateTime));
      if (lastHistorySaveTime.year() == datetime.year() && lastHistorySaveTime.month() == datetime.month() && lastHistorySaveTime.day() == datetime.day())
      {
        char keySequence[15] = "";
        String("HistoryEntry" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
        preferences.getBytes(keySequence, (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
      }
      else {
        currentSavePosition ++;
      }

      saveHistoryEntry.date.day = datetime.day();
      saveHistoryEntry.date.month = datetime.month();
      saveHistoryEntry.date.year = datetime.year();
      saveHistoryEntry.busyTime[datetime.hour()].busy |= (1 << (datetime.minute() / 5));

      if (currentSavePosition == CRASH_SAVE_LIMIT) {
        currentSavePosition = 0;
      }

      // Save the history
      char keySequence[15] = "";
      String("HistoryEntry" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
      preferences.putBytes(keySequence, (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
      // Save the last time
      preferences.putBytes("LastHistorySaveTime", (void *)&datetime, sizeof(DateTime));
    }
  }
}

// Decide whether the chair is crashed or not
void decideCrashState()
{
  if ((abs(ax) > CRASH_VALUE_THR) || (abs(ay) > CRASH_VALUE_THR) || (abs(az) > CRASH_VALUE_THR))
  {
    // crashed
    // get the crash history save position
    uint8_t currentSavePosition = preferences.getChar("CrashSavePosition", 0);

    DateTime datetime = rtc.now();
    crashData.date.day = datetime.day();
    crashData.date.month = datetime.month();
    crashData.date.year = datetime.year();
    crashData.MinuteOfDay = (datetime.hour() * 60 + datetime.minute());
    crashData.AccelerationVector = (abs(ax) > abs(ay)) ? (abs(ax) > abs(az) ? abs(ax) : abs(az)) : (abs(ay) > abs(az) ? abs(ay) : abs(az));

    currentSavePosition ++;
    if (currentSavePosition == CRASH_SAVE_LIMIT)
      currentSavePosition = 0;
    // Save the crash data
    char keySequence[15] = "";
    String("CrashHistory" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
    preferences.putBytes(keySequence, (void *)&crashData, sizeof(Crash));

    // save the crash saving position
    preferences.putChar("CrashSavePosition", currentSavePosition);
  }
}

void decideTiltHistory()
{
  DateTime datetime = rtc.now();

  // Save History, wake from the timer
  uint8_t currentSavePosition = preferences.getChar("HistorySavePosition", 0);
  preferences.getBytes("LastHistorySaveTime", (void *)&lastHistorySaveTime, sizeof(DateTime));
  if (lastHistorySaveTime.year() == datetime.year() && lastHistorySaveTime.month() == datetime.month() && lastHistorySaveTime.day() == datetime.day())
  {
    char keySequence[15] = "";
    String("HistoryEntry" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
    preferences.getBytes(keySequence, (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
  }
  else
    currentSavePosition ++;

  saveHistoryEntry.date.day = datetime.day();
  saveHistoryEntry.date.month = datetime.month();
  saveHistoryEntry.date.year = datetime.year();


  // Save Tilt of History entry
  if (pitch >= 0)
  {
    if (saveHistoryEntry.tilt[datetime.hour()][datetime.minute() / 5].positive < 255)
    {
      saveHistoryEntry.tilt[datetime.hour()][datetime.minute() / 5].positive++;
    }
  }
  else
  {
    if (saveHistoryEntry.tilt[datetime.hour()][datetime.minute() / 5].negative < 255)
    {
      saveHistoryEntry.tilt[datetime.hour()][datetime.minute() / 5].negative++;
    }
  }

  saveHistoryEntry.rot[datetime.hour()][datetime.minute() / 5].left = (uint8_t)yaw;
  saveHistoryEntry.rot[datetime.hour()][datetime.minute() / 5].right = (uint8_t)((yaw - (uint8_t)yaw) * 100);

  if (currentSavePosition == CRASH_SAVE_LIMIT) {
    currentSavePosition = 0;
  }

  // Save the history
  char keySequence[15] = "";
  String("HistoryEntry" + String(currentSavePosition, DEC)).toCharArray(keySequence, 15);
  preferences.putBytes(keySequence, (void *)&saveHistoryEntry, sizeof(saveHistoryEntry));
  // Save the last time
  preferences.putBytes("LastHistorySaveTime", (void *)&datetime, sizeof(DateTime));
}

// init ulp setting
void ulpInitialize()
{
  const ulp_insn_t program[] = {
    I_MOVI(R0, 0),			    // R0 <- 0
    I_MOVI(R1, 0),			    // R1 <- 0
    M_LABEL(1),             // label_1 => measure
    I_ADC(R2, 0, 4),        // ADC channel 4 is selected.
    I_ADDR(R1, R1, R2),     // R1 += R1
    I_ADDI(R0, R0, 1),      // R0 ++
    M_BL(1, 16),            // if (R0 < 16) goto label_1
    I_RSHR(R1, R1, 4),      // R1 = R1 / 16 => average value
    I_ADDI(R0, R1, 0),      // the average value => R0
    M_BL(2, 780),           // if (R0 < 780) goto label2
    I_HALT(),
    M_LABEL(2),             // label_2 => wakeup
    I_WAKE(),               // wake up the SOC
  };

  size_t load_addr = 0;
  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, program, &size);
  ulp_run(load_addr);
}

void getGres() {
  switch (gScale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (aScale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}

void readAccelData(int16_t* destination)
{
  uint8_t rawData[6];												                  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);		// Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]);	// Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void readGyroData(int16_t* destination)
{
  uint8_t rawData[6];												                  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);		// Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]);	// Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

int16_t readTempData()
{
  uint8_t rawData[2];												                  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);			// Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0]) << 8 | rawData[1];					    // Turn the MSB and LSB into a 16-bit value
}

// Configure the motion detection control for low power accelerometer mode
void lowPowerAccelOnlyMPU6050()
{
  // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
  // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
  // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
  // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
  // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

  uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30);		// Clear sleep and cycle bits [5:6]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c | 0x30);		  // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38);		// Clear standby XA, YA, and ZA bits [3:5]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c | 0x00);		  // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07);	// Clear high-pass filter bits [2:0]

  // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | 0x00);		// Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

  c = readByte(MPU6050_ADDRESS, CONFIG);
  writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07);			  // Clear low-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, CONFIG, c | 0x00);			    // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

  c = readByte(MPU6050_ADDRESS, INT_ENABLE);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);		// Clear all interrupts
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);			    // Enable motion threshold (bits 5) interrupt only

  // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
  // for at least the counter duration
  writeByte(MPU6050_ADDRESS, MOT_THR, 0x80);				    // Set motion detection to 0.256 g; LSB = 2 mg
  writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01);				    // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

  delay(100);												// Add delay for accumulation of samples

  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07);	// Clear high-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | 0x07);		// Set ACCEL_HPF to 7; hold the initial accleration value as a referance

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7);		// Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c | 0x47);		  // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20);		// Clear sleep and cycle bit 5
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c | 0x20);		  // Set cycle bit 5 to begin low power accelerometer motion interrupts
}

void initMPU6050()
{
  // wake up device-don't need this here if using calibration function below
  // writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  // delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

  // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);			        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);			        // Use a 200 Hz rate; the same rate set in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0);			  // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);			  // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | gScale << 3);	// Set full scale range for the gyro

  // Set accelerometer configuration
  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0);		    // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);		    // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | aScale << 3);	// Set full scale range for the accelerometer

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);               // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float* dest1, float* dest2)
{
  uint8_t data[12];								// data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packetCount, fifoCount;
  int32_t gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 };

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);             // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);		    // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);	  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity = 131;				  // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;				// = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);  	// Enable FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);		  // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80);										// accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]);	// read FIFO sample count

  fifoCount = ((uint16_t)data[0] << 8) | data[1];
  packetCount = fifoCount / 12;							            // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packetCount; ii++) {
    int16_t accelTemp[3] = { 0, 0, 0 }, gyroTemp[3] = { 0, 0, 0 };
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]);	          // read data for averaging

    accelTemp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);	// Form signed 16-bit integer for each sample in FIFO
    accelTemp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accelTemp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyroTemp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyroTemp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyroTemp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    accelBias[0] += (int32_t)accelTemp[0];      // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accelBias[1] += (int32_t)accelTemp[1];
    accelBias[2] += (int32_t)accelTemp[2];
    gyroBias[0] += (int32_t)gyroTemp[0];
    gyroBias[1] += (int32_t)gyroTemp[1];
    gyroBias[2] += (int32_t)gyroTemp[2];
  }

  accelBias[0] /= (int32_t)packetCount;					// Normalize sums to get average count biases
  accelBias[1] /= (int32_t)packetCount;
  accelBias[2] /= (int32_t)packetCount;
  gyroBias[0] /= (int32_t)packetCount;
  gyroBias[1] /= (int32_t)packetCount;
  gyroBias[2] /= (int32_t)packetCount;

  if (accelBias[2] > 0L) {
    accelBias[2] -= (int32_t)accelsensitivity;		// Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accelBias[2] += (int32_t)accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyroBias[0] / 4 >> 8) & 0xFF;			// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyroBias[0] / 4) & 0xFF;						// Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyroBias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyroBias[1] / 4) & 0xFF;
  data[4] = (-gyroBias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyroBias[2] / 4) & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);					// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float)gyroBias[0] / (float)gyrosensitivity;		// construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float)gyroBias[1] / (float)gyrosensitivity;
  dest1[2] = (float)gyroBias[2] / (float)gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accelBiasReg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);				// Read factory accelerometer trim values
  accelBiasReg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accelBiasReg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accelBiasReg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL;												          // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t maskBit[3] = { 0, 0, 0 };									  // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if (accelBiasReg[ii] & mask) maskBit[ii] = 0x01;	// If temperature compensation bit is set, record that fact in maskBit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accelBiasReg[0] -= (accelBias[0] / 8);							// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accelBiasReg[1] -= (accelBias[1] / 8);
  accelBiasReg[2] -= (accelBias[2] / 8);

  data[0] = (accelBiasReg[0] >> 8) & 0xFF;
  data[1] = (accelBiasReg[0]) & 0xFF;
  data[1] = data[1] | maskBit[0];									    // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accelBiasReg[1] >> 8) & 0xFF;
  data[3] = (accelBiasReg[1]) & 0xFF;
  data[3] = data[3] | maskBit[1];									    // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accelBiasReg[2] >> 8) & 0xFF;
  data[5] = (accelBiasReg[2]) & 0xFF;
  data[5] = data[5] | maskBit[2];									    // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);			// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (float)accelBias[0] / (float)accelsensitivity;
  dest2[1] = (float)accelBias[1] / (float)accelsensitivity;
  dest2[2] = (float)accelBias[2] / (float)accelsensitivity;
}

// Accelerometer and gyroscope self test
// check calibration wrt factory settings
// should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void mpu6050SelfTest(float* destination)
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];

  // Configure the accelerometer for self-test
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0);			  // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0xE0);			  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(250);												                    // Delay a while to let the device execute the self-test
  rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X);	// X-axis self-test results
  rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y);	// Y-axis self-test results
  rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z);	// Z-axis self-test results
  rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A);	// Mixed-axis self-test results

  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4;		// XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2;		// YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03);  			// ZA_TEST result is a five-bit unsigned integer

  // Extract the gyration test results first
  selfTest[3] = rawData[0] & 0x1F;			// XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1] & 0x1F;			// YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2] & 0x1F;			// ZG_TEST result is a five-bit unsigned integer

  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[0] - 1.0) / 30.0)));		// FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[1] - 1.0) / 30.0)));		// FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[2] - 1.0) / 30.0)));		// FT[Za] factory trim calculation
  factoryTrim[3] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[3] - 1.0)));							// FT[Xg] factory trim calculation
  factoryTrim[4] = (-25.0 * 131.0) * (pow(1.046, ((float)selfTest[4] - 1.0)));						// FT[Yg] factory trim calculation
  factoryTrim[5] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[5] - 1.0)));							// FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i];		// Report percent differences
  }
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);	// Initialize the Tx buffer
  Wire.write(subAddress);				    // Put slave register address in Tx buffer
  Wire.write(data);					        // Put data in Tx buffer
  Wire.endTransmission();				    // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;								            // data will store the register data
  Wire.beginTransmission(address);			  // Initialize the Tx buffer
  Wire.write(subAddress);						      // Put slave register address in Tx buffer
  Wire.endTransmission(false);				    // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1);	// Read one byte from slave register address
  data = Wire.read();							        // Fill Rx buffer with result
  return data;								            // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);	      // Initialize the Tx buffer
  Wire.write(subAddress);				          // Put slave register address in Tx buffer
  Wire.endTransmission(false);		        // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;

  // Put read results in the Rx buffer
  Wire.requestFrom(address, count);	      // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];			    // short name local variable for readability
  float norm;													                      // vector norm
  float f1, f2, f3;											                    // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;	// objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;

  // gyro bias error
  float gerrx = 0, gerry = 0, gerrz = 0;
  float gbiasx = 0, gbiasy = 0, gbiasz = 0;

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 - (beta * hatDot1)) * deltat;
  q2 += (qDot2 - (beta * hatDot2)) * deltat;
  q3 += (qDot3 - (beta * hatDot3)) * deltat;
  q4 += (qDot4 - (beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

