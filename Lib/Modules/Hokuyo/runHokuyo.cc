#include "HokuyoCircularHardware.hh"
#include "MagicSensorDataTypes.hh"
#include "MagicStatus.hh"
#include "Timer.hh"
#include <string>
#include <vector>
#include "ErrorMessage.hh"
#include <ipc.h>
#include <stdlib.h>

using namespace std;
using namespace Upenn;
using namespace Magic;


#define HOKUYO_DEF_DEVICE "/dev/ttyACM0"
#define HOKUYO_DEF_LOG_NAME "hokuyo"
//#define HOKUYO_DEF_DEVICE "/dev/cu.usbmodem3d11"

//SCAN_NAME is used to determine the scanType and scanSkip values
//See below for details

#define SCAN_NAME "range"
//#define SCAN_NAME "top_urg_range+intensity"
//#define SCAN_NAME "range+intensity1+AGC1"

enum { HOKUYO_TYPE_UTM,
       HOKUYO_TYPE_UBG
     };

HokuyoCircularHardware * dev = NULL;

void ShutdownFn(int code)
{
  PRINT_INFO("exiting..\n");
  if (dev)
  { 
    PRINT_INFO("Stopping thread\n");
    if (dev->StopThread())
    {
      PRINT_ERROR("could not stop thread\n");
    }

    PRINT_INFO("Stopping device\n");
    if (dev->StopDevice())
    {
      PRINT_ERROR("could not stop device\n");
    }

    dev->Disconnect();
    delete dev;
  }
  exit(0);
}

int main(int argc, char * argv[])
{
  string address = string(HOKUYO_DEF_DEVICE);
  string ipcHost = string("localhost");

  string id = string();

  if (argc>=2)
    address = string(argv[1]);

  if (argc>=3)
    id = string(argv[2]);
  
  string logName          = string(HOKUYO_DEF_LOG_NAME) + id;

  char * robotIdStr       = getenv("ROBOT_ID");
  if (!robotIdStr)
  {
    printf("ROBOT_ID must be defined\n");
    return -1;
  }

  string robotName        = string("Robot") + robotIdStr;
  
  char * lidar0Type = getenv("LIDAR0_TYPE");
  char * lidar1Type = getenv("LIDAR1_TYPE");


  //default to HOKUYO_UTM
  if (!lidar0Type)
    lidar0Type = (char*)"HOKUYO_UTM";

  if (!lidar1Type)
    lidar1Type = (char*)"HOKUYO_UTM";


  int nPoints = 0;
  if (argc >=4)
    nPoints = strtol(argv[3],NULL,10);
  
  const int numBuffers=50;      //number of buffers to be used in the circular buffer
  const int bufferSize = HOKUYO_MAX_DATA_LENGTH;
  int baudRate=115200;            //communication baud rate (does not matter for USB connection)
  dev = new HokuyoCircularHardware(bufferSize,numBuffers); //create an instance of HokuyoCircular

  PRINT_INFO("Connecting to device "<< address <<"\n");
  if (dev->Connect(address,baudRate))   //args: device name, baud rate
  {
    PRINT_ERROR("could not connect\n");
    return -1;
  }

  string lidarTypeStr;
  int lidarType;
  string serial = dev->GetSerial();
  PRINT_INFO("Sensor's serial number is "<<serial<<"\n");

  if (id.empty() || id.compare("-1") == 0)
  {
    char * lidar0serial = getenv("LIDAR0_SERIAL");
    char * lidar1serial = getenv("LIDAR1_SERIAL");

    if ( !lidar0serial || !lidar1serial)
    {
      PRINT_ERROR("LIDAR0_SERIAL and/or LIDAR1_SERIAL are not defined\n");
      return -1;
    }

    if (serial.compare(lidar0serial) == 0)
    {
      id = string("0");
      lidarTypeStr = string(lidar0Type);
    }
    else if (serial.compare(lidar1serial) == 0)
    {
      id = string("1");
      lidarTypeStr = string(lidar1Type);
    }
    else
    {
      PRINT_ERROR("lidar id is not defined and current serial (" <<serial<<") does not match neither LIDAR0_SERIAL nor LIDAR1_SERIAL\n");
      return -1;
    }
    PRINT_INFO("Sensor identified as LIDAR"<<id<<"\n");
  }

  if (lidarTypeStr.compare("HOKUYO_UTM") == 0)
    lidarType = HOKUYO_TYPE_UTM;
  else if (lidarTypeStr.compare("HOKUYO_UBG") == 0)
    lidarType = HOKUYO_TYPE_UBG;
  else
  {
    lidarType = HOKUYO_TYPE_UTM;
  }

  int maxPoints;
  switch (lidarType)
  {
    case HOKUYO_TYPE_UTM:
      maxPoints = 1081;
      break; 
    case HOKUYO_TYPE_UBG:
      maxPoints = 769;
      break;
    default:
      PRINT_ERROR("unknown sensor type: " << lidarType << "\n");
      return -1;
  }

  if (nPoints ==0)
    nPoints = maxPoints;
  else if (nPoints > maxPoints)
  {
    PRINT_ERROR("nPoints is larger than maxPoints : " << nPoints << " > " << maxPoints << "\n");
    return -1;
  }

  PRINT_INFO("Number of points in scan = " << nPoints << "\n");


  string lidarScanMsgName = robotName + "/Lidar" + id;
  //connect to ipc
  string processName      = string("runHokuyo") + id; 
  IPC_connectModule(processName.c_str(),ipcHost.c_str());

  IPC_defineMsg(lidarScanMsgName.c_str(),IPC_VARIABLE_LENGTH,
                 Magic::LidarScan::getIPCFormat());

/*
  if (dev->InitializeLogging(logName))   //initialize logging
  {
    PRINT_INFO("could not initialize logging\n");
    return -1;
  }

  if (dev->EnableLogging())   //enable logging
  {
    PRINT_INFO("could not enable logging\n");
    return -1;
  }
*/

  int scanStart=0;      //start of the scan
  int scanEnd=nPoints -1;//1080;      //end of the scan
  int scanSkip=1;       
  int encoding=HOKUYO_3DIGITS; 
  int scanType;                
  char scanName[128];        //name of the scan - see Hokuyo.hh for allowed types
  strcpy(scanName,SCAN_NAME); 

  int sensorType= dev->GetSensorType();
  string firmware = dev->GetFirmware();
  PRINT_INFO("firmware: "<<firmware<<"\n");
  int newSkip;

  //get the special skip value (if needed) and scan type, depending on the scanName and sensor type
  if (dev->GetScanTypeAndSkipFromName(sensorType, scanName, &newSkip, &scanType)){
    PRINT_ERROR("Error getting the scan parameters\n");
    exit(1);
  }

  if (newSkip!=1){            //this means that a special value for skip must be used in order to request
    scanSkip=newSkip;         //a special scan from 04LX. Otherwise, just keep the above specified value of 
  }                           //skip


  //start the thread, so that the UpdateFunction will be called continously
  PRINT_INFO("Starting thread\n");
  if (dev->StartThread())
  {
    PRINT_ERROR("could not start thread\n");
    return -1;
  }


  //set the scan parameters
  if (dev->SetScanParams(scanName,scanStart, scanEnd, scanSkip, encoding, scanType)){
    PRINT_INFO("Error setting the scan parameters\n");
    exit(1);
  }


  double timeout_sec = 0.1;
  double time_stamp;

  vector< vector<unsigned int> > values;
  vector<double> timeStamps;


  //fill the lidarScan static values
  Magic::LidarScan lidarScan;
  lidarScan.ranges.size = nPoints;
  lidarScan.ranges.data = new float[lidarScan.ranges.size];
  lidarScan.startAngle  = -135.0/180.0*M_PI;;
  lidarScan.stopAngle   = 135.0/180.0*M_PI;;
  lidarScan.angleStep   = 0.25/180.0*M_PI;
  lidarScan.counter     = 0;
  lidarScan.id          = 0;

  HeartBeatPublisher hBeatPublisher;
  if ( hBeatPublisher.Initialize((char*)processName.c_str(),(char*)lidarScanMsgName.c_str()) )
  {
    PRINT_ERROR("could not initialize the heartbeat publisher\n");
    return -1;
  }

  //capture CTRL-C for proper shutdown
  signal(SIGINT,ShutdownFn);

  Timer scanTimer;
  scanTimer.Tic();
  int cntr =0;

  while(1)
  {
    if (dev->GetValues(values,timeStamps,timeout_sec) == 0)
    {
      int numPackets = values.size();
      //printf("num packets = %d\n",numPackets);
      for (int j=0; j<numPackets; j++)
      {
        cntr++;
        time_stamp = timeStamps[j];
        
        //copy ranges
        vector<unsigned int> & ranges = values[j];
        
        //fill the LidarScan packet
        lidarScan.startTime = lidarScan.stopTime = time_stamp;
        lidarScan.counter++;
        
        float * rangesF = lidarScan.ranges.data;
        for (unsigned int jj=0;jj<lidarScan.ranges.size; jj++)
          *rangesF++ = (float)ranges[jj]*0.001;


        //publish messages
        IPC_publishData(lidarScanMsgName.c_str(),&lidarScan);

        //publis heart beat message
        if (lidarScan.counter % 40 == 0)
        {
          if (hBeatPublisher.Publish(0))
          {
            PRINT_ERROR("could not publish heartbeat\n");
            //return -1;
          }
        }

        //printf(".");fflush(stdout);
        if( cntr % 40 == 0)
        {
          double dt = scanTimer.Toc(); scanTimer.Tic();
          printf("scan rate = %f\n",40/dt);
        }
      }
    }

    else
    {
      PRINT_ERROR("could not get values (timeout)\n");
    }
  } 

  return 0;
}
