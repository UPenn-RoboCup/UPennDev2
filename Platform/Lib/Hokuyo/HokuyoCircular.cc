/*
  Buffered driver/logger for Hokuyo laser range scanners
  
  Alex Kushleyev, 10/2009
  University of Pennsylavania
  akushley@seas.upenn.edu
*/

#include "HokuyoCircular.hh"
using namespace Upenn;

///////////////////////////////////////////////////////////////////
// Constructor
HokuyoCircular::HokuyoCircular(int bufferSize, int numBuffers)
              : DeviceInterface(bufferSize,numBuffers)
{
  this->scanSettingsChanged = false;
  this->needToStopLaser     = false;
  this->active              = false;
  this->connected           = false; 
}

///////////////////////////////////////////////////////////////////
// Destructor
HokuyoCircular::~HokuyoCircular()
{
}

///////////////////////////////////////////////////////////////////
// Pause the sensor operation
int HokuyoCircular::PauseSensor()
{
  printf("entered SetScanParams... waiting to lock the mutex");
  LockSettingsMutex();
  printf("settings locked");
  
  this->active          = false;
  this->needToStopLaser = true;
  
  UnlockSettingsMutex();
  printf("settings unlocked");
  return 0;
}

///////////////////////////////////////////////////////////////////
// Resume sensor operation
int HokuyoCircular::ResumeSensor()
{
  printf("entered SetScanParams... waiting to lock the mutex");
  LockSettingsMutex();
  printf("settings locked");
  
  this->active = true;
  
  UnlockSettingsMutex();
  printf("settings unlocked");
  return 0;
}

///////////////////////////////////////////////////////////////////
// Set scan parameters (default implementation)
int HokuyoCircular::SetScanParams(char * scanTypeNameNew,int scanStartNewNew, 
                                  int scanEndNewNew, int scanSkipNewNew, 
                                  int encodingNewNew, int scanTypeNewNew)
{
  printf("function is not implemented!!");
  return 0;
}

///////////////////////////////////////////////////////////////////
// Get the values from circular buffer (independent of implementation)
int HokuyoCircular::GetValues(vector< vector<unsigned int> > & values, 
                              vector<double> & time_stamps,
                              double timeout_sec)
{
  if (!this->IsThreadRunning())
  {
    printf("main thread is not running.\n");
    return -1;
  } 

  char * read_ptr;
  int data_length;
  double time_stamp;
  int n_packets_remaining=1;
  int n_packets_got = 0;

  //clear out the output variables
  values.clear();
  time_stamps.clear();

    
  //read off all the data from the ring buffers
  while (n_packets_remaining > 0)
  {
    if (this->GetReadPtr((const char **)(&read_ptr),data_length,time_stamp,timeout_sec)==0)
    {
      if ( (data_length <= 0) || (data_length % sizeof(unsigned int) != 0) )
      {      
        cout << "bad packet length: " << data_length << " chars" <<endl;
        this->DoneReading();
        return -1;
      }

      unsigned int nData = data_length / sizeof(unsigned int);
      vector<unsigned int> v;
      values.push_back(v);
      values.back().resize(nData);
      
      memcpy(&(values.back()[0]),read_ptr,data_length);


      this->DoneReading(&n_packets_remaining);
       
      //store the timestamp
      time_stamps.push_back(time_stamp);
      n_packets_got++;
    }

    else
    {
      printf("could not get a pointer to the packet\n");
      return -1;
    }
  }

  return 0;
}

