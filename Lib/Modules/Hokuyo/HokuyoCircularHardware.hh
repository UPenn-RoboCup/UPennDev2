/*
  Buffered driver/logger for Hokuyo laser range scanners
  
  Alex Kushleyev, 10/2009
  University of Pennsylavania
  akushley@seas.upenn.edu
*/

#ifndef HOKUYO_CIRCULAR_HARDWARE_HH
#define HOKUYO_CIRCULAR_HARDWARE_HH

#include <string>
#include <vector>
#include "HokuyoCircular.hh"
#include "DataLoggerQueueDB.hh"


#define HOKUYO_CIRCULAR_SENSOR_NUM_STOP_LASER_RETRIES 3
#define HOKUYO_CIRCULAR_SENSOR_MAX_NUM_ERRORS_BEFORE_RESTART 3
#define HOKUYO_CIRCULAR_SENSOR_GET_SCAN_TIMEOUT 200
#define HOKUYO_CIRCULAR_SENSOR_SET_SCAN_PARAMS_TIMEOUT 500

using namespace std;

namespace Upenn
{
  class HokuyoCircularHardware : public HokuyoCircular
  {
    //constructor
    public: HokuyoCircularHardware(int bufferSize, int numBuffers);  

    //destructor
    public: virtual ~HokuyoCircularHardware();

    //this function will do the reading from serial port into buffers
	  private: int GetData(char * writePtr, int maxLength, int & dataLength, 
                        double & timestamp, bool needToLog = false);


    //connec to to the sensor
    public: int Connect(string dev, int baudRate);
    
    //disconnect from the sensor
    public: int Disconnect();

    //set the scan parameters for the next scan
    public: int SetScanParams(char * newScanType, int newScanStart, int newScanEnd, 
                              int newScanSkip, int newEncoding, int newSpecialScan);

    //get scan params that should be sent to sensor in order to get a specific scan
    public: int GetScanTypeAndSkipFromName(int sensorType, char * scanTypeName, int * skip, int * scanType);
    
    //get the sensor type
    public: int GetSensorType();

    //get the sensor serial number
    public: std::string GetSerial();
    public: std::string GetFirmware();

    //store the latest scan settings
    public: int scanStartNew, scanEndNew, scanSkipNew, encodingNew, scanTypeNew;
    
    //pointer to the driver
    private: Hokuyo * hokuyo;
    
    //string name of the current scan type
    private: string scanTypeName; //not sure whether this is actually needed
    
  };
}
#endif //HOKUYO_CIRCULAR_HARDWARE_HH
