/*
 * Created in 2018/1/10
 * Modified in 2018/1/15 5:15  
 * Structures for BLE
 * Author - Butterfly
 */
 
struct QuaternionBLE
{
	int16_t W;
	int16_t X;
	int16_t Y;
	int16_t Z;
};

struct Date
{
	uint8_t day : 5;
	uint8_t month : 4;
	uint8_t year : 7;
};

struct Time
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};

struct DateTimeData
{
	Date date;
	Time tm;
};

struct BusyTime
{
	uint16_t busy : 12;
};

struct Crash
{
	Date date;
	uint16_t MinuteOfDay : 11;
	uint8_t AccelerationVector : 5;
};

struct Rotation
{
	uint8_t left : 7;
	uint8_t right : 7;
};

struct Tilt
{
	uint8_t positive;
	uint8_t negative;
};

struct HistoryInfo
{
	DateTimeData beginTm;
	DateTimeData endTime;
};

struct RequestHistory
{
	DateTimeData beginTm;
	DateTimeData endTm;
};

struct FetchHistory
{
	uint16_t offset;
	uint8_t* data;
};

struct CrashInfo
{
	DateTimeData beginTm;
	DateTimeData endTm;
};

struct RequestCrash
{
	DateTimeData beginTm;
	DateTimeData endTm;
};

struct FetchCrashBegin
{
	uint8_t signature = 0xAA;
	DateTimeData endTm;
	uint16_t lengthBytes;
	uint32_t CRC32OfData;
};

struct FetchCrash
{
	uint16_t offset;
	uint8_t* data;
};

struct HistoryEntry
{
	Date date;
	BusyTime busyTime[24];
	Rotation rot[24][12];
	Tilt tilt[24][12];
};

