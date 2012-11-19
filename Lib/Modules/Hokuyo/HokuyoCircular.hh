/*
  Buffered driver/logger for Hokuyo laser range scanners
  
  Alex Kushleyev, 10/2009
  University of Pennsylavania
  akushley@seas.upenn.edu
*/

#ifndef HOKUYO_CIRCULAR_HH
#define HOKUYO_CIRCULAR_HH

#include <string>
#include "Hokuyo.hh"
#include <vector>
#include "DeviceInterface.hh"
#include "DataLogger.hh"

using namespace std;

namespace Upenn
{
  class HokuyoCircular : public DeviceInterface
  {

    //public: HokuyoCircular(){};
    public: HokuyoCircular(int bufferSize, int numBuffers);  

    public: virtual ~HokuyoCircular();

    //default function for implementations that dont have this functionality
    public: virtual int SetScanParams(char * newScanType, int newScanStart, 
                                      int newScanEnd, int newScanSkip, 
                                      int newEncoding, int newSpecialScan);

	  //get all of the values from the circular buffer
    public: virtual int GetValues(vector< vector<unsigned int> > & values, 
                                  vector<double> & time_stamps,
                                  double timeout_sec);

    //pause sensor operation
    public: virtual int PauseSensor();
    
    //resume sensor operation
    public: virtual int ResumeSensor();

    //variables common to all implementations
    protected: bool active;
    protected: bool connected;
    protected: bool scanSettingsChanged;
    protected: bool needToStopLaser;
  };
}
#endif //HOKUYO_CIRCULAR_HH
