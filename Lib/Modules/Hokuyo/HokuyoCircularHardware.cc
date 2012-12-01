/*
  Buffered driver/logger for Hokuyo laser range scanners
  
  Alex Kushleyev, 10/2009
  University of Pennsylavania
  akushley@seas.upenn.edu
*/

#include "HokuyoCircularHardware.hh"

#include "ErrorMessage.hh"

using namespace Upenn;

///////////////////////////////////////////////////////////////////
// Constructor
HokuyoCircularHardware::HokuyoCircularHardware(int bufferSize, int numBuffers)
              : HokuyoCircular(bufferSize,numBuffers)
{
  this->hokuyo              = NULL;
  
  //default scan type
  this->scanTypeName        = string("range");
  
}

///////////////////////////////////////////////////////////////////
// Destructor
HokuyoCircularHardware::~HokuyoCircularHardware()
{
  this->Disconnect();
  if (this->hokuyo) delete this->hokuyo;
}


///////////////////////////////////////////////////////////////////
// Connect to the device
int HokuyoCircularHardware::Connect(string dev, int baudRate)
{
  if (this->connected)
    return 0;

  //create instance of the Hokuyo driver
  if (!this->hokuyo)
  {
    this->hokuyo = new Hokuyo();

    if (!this->hokuyo)
    {
      PRINT_ERROR( "Unable to create instance of Hokuyo!!!\n" );
      return -1;
    }
  }

  //connect using Hokuyo driver
  if (this->hokuyo->Connect(dev.c_str(),baudRate))
  {
    PRINT_ERROR( "Unable to initialize Hokuyo!!!\n" );
    return -1;
  }

  this->connected=true; 

  return 0;
}


///////////////////////////////////////////////////////////////////
// Disconnect from device
int HokuyoCircularHardware::Disconnect()
{ 
  if (this->connected)
  {
    PRINT_INFO("Disconnecting from device...");
    this->hokuyo->Disconnect();
    PRINT_INFO(" done\n"); 
    this->connected=false;
  }
  return 0;
}

///////////////////////////////////////////////////////////////////
// Implementation of DeviceInterface's GetData function
// Reads data from the sensor and writes it to the locked
// pointer in the circular buffer
int HokuyoCircularHardware::GetData(char * writePtr, int maxLength, 
                                    int & dataLength, double & timestamp,
                                    bool needToLog)
{
  int  scanStartCurr;
  int  scanEndCurr;
  int  scanSkipCurr;
  int  encodingCurr;
  int  scanTypeCurr;
  bool connectedCurr;
  bool activeCurr;

  //reset the data length
  dataLength = 0;
  timestamp =-1;
    
  LockSettingsMutex();

  //create a local copy of the scan params to be used later=
  scanStartCurr   = this->scanStartNew;
  scanEndCurr     = this->scanEndNew;
  scanSkipCurr    = this->scanSkipNew;
  encodingCurr    = this->encodingNew;
  connectedCurr   = this->connected;
  activeCurr      = this->active;
  scanTypeCurr    = this->scanTypeNew;

  //double check the scan settings
  if ( (this->scanStartNew != this->hokuyo->GetScanStart() ) || 
       (this->scanEndNew   != this->hokuyo->GetScanEnd()   ) || 
       (this->scanSkipNew  != this->hokuyo->GetScanSkip()  ) || 
       (this->encodingNew  != this->hokuyo->GetEncoding()  ) ||
       (this->scanTypeNew  != this->hokuyo->GetScanType()  )  )
  {
    this->scanSettingsChanged=true;

    if (this->hokuyo->GetStreaming())
    {      
      this->needToStopLaser=true;
    }
  }

  //if the scan settings changed, we dont want to use the previous scan,
  //since it will contain data from old sensor params
  if (this->scanSettingsChanged)
  {
    this->EmptyCircularBuffer();
  }

  //shut off the laser if needed (stops the stream)
  if (this->needToStopLaser)
  {
    if (this->hokuyo->LaserOff() == 0)
    {
      PRINT_ERROR("Laser has been shut off\n");
      this->needToStopLaser=false;
    }
      
    else 
    {
      PRINT_ERROR("Wasn't able to shut off the laser\n");
    }
  }
    
  this->scanSettingsChanged=false;
  this->SettingsCondSignal();
  this->UnlockSettingsMutex();
    
  //PRINT_ERROR("reader: settings locked");
  if (this->connected && this->active)
  {
    int numData;
    if ( this->hokuyo->GetScan((unsigned int *)writePtr, numData, scanStartCurr, 
                               scanEndCurr, scanSkipCurr, encodingCurr,scanTypeCurr, 0) == 0)
    {
      //fill in the remaining output variables
      timestamp = this->GetAbsoluteTime();
      dataLength = numData * sizeof(unsigned int);
      return 0;
    } 

    else //didn't get a scan
    {
      dataLength = 0;
      PRINT_ERROR( "Could not read a scan from the sensor\n" );

      //something happened - got out of sync? resynchronize.
      if ( this->hokuyo->FindPacketStart() == 0 )
      {
        PRINT_WARNING( "Resynchronized with the stream.\n" );
      }
    }
  }
    
  else //not connected to the device
  {
    if (!this->connected)
    {
      PRINT_ERROR( "not connected\n" );
      return -1;
    }
  }

  return 0;
}

///////////////////////////////////////////////////////////////////
// Set the scanner parameters
int HokuyoCircularHardware::SetScanParams(char * scanTypeNameNew,int scanStartNewNew, 
                                  int scanEndNewNew, int scanSkipNewNew, 
                                  int encodingNewNew, int scanTypeNewNew)
{
  //PRINT_INFO("entered SetScanParams... waiting to lock the mutex\n");
  LockSettingsMutex();
  //PRINT_INFO("settings locked\n");
  
  //copy the new values
  this->scanStartNew        = scanStartNewNew;
  this->scanEndNew          = scanEndNewNew;
  this->scanSkipNew         = scanSkipNewNew;
  this->encodingNew         = encodingNewNew;
  this->scanTypeNew         = scanTypeNewNew;
  this->needToStopLaser     = true;
  this->scanTypeName        = string(scanTypeNameNew);
  this->active              = true;
  this->scanSettingsChanged = true;

  timeval timeStart,timeStop;
  gettimeofday(&timeStart,NULL);
  gettimeofday(&timeStop,NULL);
  double waitedTime=(timeStop.tv_sec+timeStop.tv_usec*0.000001)-(timeStart.tv_sec+timeStart.tv_usec*0.000001);

  //PRINT_INFO("set the settings.. waiting for the next scan read before returning\n" );

  //this loop makes sure that timedwait does not return before the time expires - recommended in manual for this function..
  while ( (waitedTime*1000 < HOKUYO_CIRCULAR_SENSOR_SET_SCAN_PARAMS_TIMEOUT) && (this->scanSettingsChanged) )
  {         
    //int ret=SettingsCondWait(&maxWaitTime);    
    int ret=SettingsCondWait(HOKUYO_CIRCULAR_SENSOR_SET_SCAN_PARAMS_TIMEOUT);
    if (ret==ETIMEDOUT)
    {
      PRINT_ERROR("timeout!!!!\n");
    }

    gettimeofday(&timeStop,NULL);
    waitedTime=(timeStop.tv_sec+timeStop.tv_usec*0.000001)-(timeStart.tv_sec+timeStart.tv_usec*0.000001);
    if ( (waitedTime*1000 < HOKUYO_CIRCULAR_SENSOR_SET_SCAN_PARAMS_TIMEOUT) && (this->scanSettingsChanged) )
    {      
      usleep(1000);
    }
  }
  
  //need to make sure that the settings changed
  if (this->scanSettingsChanged)
  {
    PRINT_ERROR( "could not set the parameters!! Waited time="<<waitedTime<<"\n" );
    UnlockSettingsMutex();
    return -1;
  }

  UnlockSettingsMutex();
  //PRINT_INFO("waited time="<< waitedTime<<"\n");
  //PRINT_INFO("settings unlocked\n");
  
  return 0;
}


///////////////////////////////////////////////////////////////////
// Convert a string scan type to the integer type for use with driver
int HokuyoCircularHardware::GetScanTypeAndSkipFromName(int sensorType, char * scanTypeName, int * skip, int * scanType)
{
  return Hokuyo::GetScanTypeAndSkipFromName(sensorType, scanTypeName, skip, scanType);
}


///////////////////////////////////////////////////////////////////
// Get the sensor type from the low-level driver
int HokuyoCircularHardware::GetSensorType()
{
  if (!this->hokuyo)
  {
    PRINT_ERROR("hokuyo is not initialized\n");
    return -1;
  }
  return this->hokuyo->GetSensorType();
}

///////////////////////////////////////////////////////////////////
// Get the sensor type from the low-level driver
std::string HokuyoCircularHardware::GetSerial()
{
  if (!this->hokuyo)
  {
    PRINT_ERROR("hokuyo is not initialized\n");
    return string();
  }
  return string(this->hokuyo->GetSerial());
}

///////////////////////////////////////////////////////////////////
// Get the sensor type from the low-level driver
std::string HokuyoCircularHardware::GetFirmware()
{
  if (!this->hokuyo)
  {
    PRINT_ERROR("hokuyo is not initialized\n");
    return string();
  }
  return string(this->hokuyo->GetFirmware());
}


