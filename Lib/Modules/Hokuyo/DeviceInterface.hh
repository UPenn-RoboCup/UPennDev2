#ifndef DEVICE_INTERFACE_HH
#define DEVICE_INTERFACE_HH

#include "CircularBuffer.hh"
//#include "DataLoggerQueueDB.hh"
#include <pthread.h>
#include <string>
#include <ipc.h>
#include "Timer.hh"

#define DEVICE_INTERFACE_UPDATE_FUNCTION_ERROR_SLEEP_US 10000
#define DEVICE_INTERFACE_MAX_NUM_CONSECUTIVE_ERRORS 10
#define DEVICE_INTERFACE_MAX_TEMP_BUF_SIZE 1000000000

#define DEVICE_INTERFACE_TEMP_LOG_SIZE_MULT 10
#define DEVICE_INTERFACE_DEF_LOG_TYPE 0


namespace Upenn
{
//  typedef DataLoggerQueueDB DATA_LOGGER_TYPE;

  class DeviceInterface
  {
    public: DeviceInterface(int bufferLength, int numBuffers);
    public: virtual ~DeviceInterface();

    public: int StartThread();
    public: int StopThread();
    public: int RunSingleUpdate();
    public: virtual int StartDevice();
    public: virtual int StopDevice();
    
//    public: virtual int InitializeLogging(std::string filename);
//    public: virtual int EnableLogging();
//    public: virtual int DisableLogging();
    public: int SetDeviceID(int id);

//    protected: virtual DATA_LOGGER_TYPE * CreateLogger();


    protected: int LockSettingsMutex();
    protected: int UnlockSettingsMutex();
//    protected: bool GetLoggingEnabled();
//    protected: bool GetLoggingInitialized();

    protected: bool IsThreadRunning();
    private: virtual int UpdateFunction();
    private: virtual int GetData(char * writePtr, int maxLength, 
                                 int & dataLength, double & timestamp,
                                 bool needToLog = false) = 0;

//    private: virtual int FreeLogMem();
                     
    protected: int   GetReadPtr( const char ** dataPtrPtr, int & dataLength, 
                                 double & timeStamp, double timeoutSec);

    protected: int   DoneReading( int * numPacketsRemaining = NULL );

    protected: double GetAbsoluteTime();
	  protected: int SettingsCondSignal();
	  protected: int  SettingsCondWait(int timeouMs);
    private: pthread_cond_t settingsCond;

	  private: pthread_t thread;
    private: bool deviceStarted;
    private: static void *ThreadFunc(void * input);

    protected: int EmptyCircularBuffer();

    private: bool threadRunning;
    private: pthread_mutex_t settingsMutex;
    protected: CircularBuffer * cBuf;
//    protected: DATA_LOGGER_TYPE * logger;
//    private: bool loggingInitialized;
//    private: bool loggingEnabled;
//    private: string writeLogFileName;
//    private: int tempLogSizeMult;
//    protected: int logType;
//    protected: string logFormat;
//    protected: FORMATTER_PTR logFormatter;
//    protected: string logInfoHeader;
    protected: int id;
  };
}

#endif //DEVICE_INTERFACE_HH
