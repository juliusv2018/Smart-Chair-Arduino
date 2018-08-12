/*
 * Created in 2018/1/13 22:39
 * Modified in 2018/1/15 05:40  
 * Declaration of variables and functions
 * Author - Butterfly
 */

/////////////////////////////////////////////////////////////////////////////////////////////
//                                     MACRO DECLARATION                                   //
/////////////////////////////////////////////////////////////////////////////////////////////
// I2C device address
#define MPU6050_DEV_ADDRESS		0x69
#define PCF8523_ADDRESS			  0x68
#define MPU6050_PWR_MGMT_2		0x6C

// PIN Number
#define SDA_PIN         21          // I2C PIN 33
#define SCL_PIN         22          // I2C PIN 36
#define MPU_INT_PIN     25          // IN PIN 10
#define RTC_INT_PIN     19          // IN PIN 31
#define CHARGESTATE_PIN 23          // IN PIN 37
#define CHARGELED_PIN    5          // OUT PIN 29
#define LDOEnable       26          // OUT PIN 11
#define ADCEnable       27          // OUT PIN 12
#define ADCBattery      32          // ADC IN PIN 8

// define low power state and realtime state
#define INITIAL_STATE		      0         // The state when the power is on
#define LOW_POWER_STATE		  	1         // The state when the ESP32 goes to the low power mode
#define REALTIME_STATE		  	2         // The state when the ESP32 is waked up by MPU6050 External Interrupt(EXT1)
#define LOW_BATTERY_STATE	  	3         // ULP Wake up
#define SAVE_HISTORY_STATE		4         // The state when the ESP32 is waked up by Timer Wake up for saving history
#define EXT1_AWAKE_STATE		  5

// Busy time parameters
#define BUSY_TIME_VALUE_THR       70    // the threshold of accelermeter for busy time
#define BUSY_TIME_DURATION_HTHR   300   // The upeer threshold of duration for busy time
#define BUSY_TIME_DURATION_LTHR   10    // The lower threshold of duration for busy time
#define CRASH_VALUE_THR           100   // the threshold of crash

#define uS_TO_S_FACTOR            1000000   // Conversion factor for micro seconds to seconds
#define NO_MOTION_SECONDS_TIME		120		    // 120 seconds
#define CRASH_SAVE_LIMIT          10        // limit for the crash saving
#define HISTORY_SAVE_LIMIT        10        // limit for the history saving



/////////////////////////////////////////////////////////////////////////////////////////////
//                                   VARIABLE DECLARATION                                  //
/////////////////////////////////////////////////////////////////////////////////////////////
// Specify sensor full scale
int gScale = GFS_250DPS;
int aScale = AFS_2G;
float aRes, gRes;		    // scale resolutions per LSB for the sensors

// Pin definitions
int blinkPin = 13;		  // Blink LED on Teensy or Pro Mini when updating
int intPin = 25;		    // These can be changed, 2 and 3 are the Arduinos ext int pins
boolean blinkOn = false;

// Bias corrections for gyro and accelerometer
float flGyroBias[3] = { 0, 0, 0 };
float flAccelBias[3] = { 0, 0, 0 };

float ax, ay, az;       // Stores the real accel value in g's
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t tempCount;      // Stores the real internal chip temperature in degrees Celsius

float SelfTest[6];
uint32_t deltT = 0;     // used to control display output rate
uint32_t count = 0;		  // used to control display output rate

// Parameters for 6 DoF sensor fusion calculations
float gyroMeasError = PI * (40.0f / 180.0f);		  // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float gyroMeasDrift = PI * (2.0f / 180.0f);       // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * gyroMeasError;		// compute beta
float zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift;		// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float pitch, yaw, roll;
float deltat = 0.0f;								              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;			    // used to calculate integration interval
uint32_t Now = 0;									                // used to calculate integration interval
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };          // vector to hold quaternion

// MPU6050 and RTC Device
MPU6050 mpu(MPU6050_DEV_ADDRESS);
RTC_DS1307 rtc;
// RTC_PCF8523 rtc;

// For measuring accelerrometer and gyroscope
int16_t gyroX, gyroY, gyroZ;
int16_t accelX, accelY, accelZ;

// Some variables for busytime detection
float busytimeOldAx, busytimeOldAy, busytimeOldAz;
bool nowBusy = false;

// time variable for detecting no motion - time when the interrupt occurs
DateTime busytimeOldTime;
DateTime intOldTime;
DateTime gyroActiveSecondsTime;

// variables for saving history
HistoryEntry saveHistoryEntry;
Crash crashData;

// state
int state = INITIAL_STATE;
uint8_t bleOldTime;

// for saving to the flash
Preferences preferences;
DateTime lastCrashSaveTime;
DateTime lastHistorySaveTime;


/////////////////////////////////////////////////////////////////////////////////////////////
//                                  FUNCTION DECLARATION                                   //
/////////////////////////////////////////////////////////////////////////////////////////////
void dataReady();
void mpuInitialize();
void deviceInitialize();
void mpuGetData();
void variableInitialize();
void printWakeupReason();
void lowPowerModeSetting();
void realtimeModeSetting();
void decideBusyTime();
void decideCrashState();
void ulpInitialize();
void getGres();
void getAres();
void readAccelData(int16_t* destination);
void readGyroData(int16_t* destination);
void lowPowerAccelOnlyMPU6050();
void initMPU6050();
void calibrateMPU6050(float* dest1, float* dest2);
void mpu6050SelfTest(float* destination);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
uint8_t readByte(uint8_t address, uint8_t subAddress);
int16_t readTempData();
