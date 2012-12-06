#include "DeviceInterface.hh"
#include "ErrorMessage.hh"
//#include "DataLoggerQueueDB.hh"
#include "PointerQueueBuffer.hh"
#include <signal.h>
#include <unistd.h>

using namespace Upenn;

DeviceInterface::DeviceInterface(int bufferLength, int numBuffers)
{
	this->threadRunning           = false;
//  this->loggingInitialized      = false;
//  this->loggingEnabled          = false;
  this->cBuf                    = new CircularBuffer(bufferLength, numBuffers);
//  this->logger                  = NULL;
//  this->tempLogSizeMult         = DEVICE_INTERFACE_TEMP_LOG_SIZE_MULT;
//  this->logType                 = DEVICE_INTERFACE_DEF_LOG_TYPE;
  this->id                      = 0;

  pthread_mutex_init( &(this->settingsMutex) , NULL );
  pthread_cond_init( &(this->settingsCond),NULL);
}

DeviceInterface::~DeviceInterface()
{
  ///stop the thread and shutdown the device
  this->StopThread();
  this->StopDevice();
  
//  if (this->logger)  delete this->logger;
  if (this->cBuf)  delete this->cBuf;

  pthread_mutex_destroy(&(this->settingsMutex));
  pthread_cond_destroy(&(this->settingsCond));
}

//start the main thread
int DeviceInterface::StartThread()
{
  if (this->threadRunning)
  {
    PRINT_ERROR("Thread is already running\n");
    return -1;
  }
	
	PRINT_INFO("Starting thread...");

  if (pthread_create(&this->thread, NULL, this->ThreadFunc, (void *)this))
  {
    PRINT_ERROR("Could not start thread\n");
    return -1;
  }
  PRINT_INFO("done\n");

	this->threadRunning=true;
	return 0;
}

int DeviceInterface::StopThread()
{
	if (this->threadRunning)
  {
    PRINT_INFO("Stopping thread...");
    pthread_cancel(this->thread);
    pthread_join(this->thread,NULL);
    PRINT_INFO("done\n"); 
    this->threadRunning=false;
  }

	return 0;
}

//dummy function for running the main loop
void *DeviceInterface::ThreadFunc(void * arg_in)
{
	sigset_t sigs;
	sigfillset(&sigs);
	pthread_sigmask(SIG_BLOCK,&sigs,NULL);

	DeviceInterface * di = (DeviceInterface *) arg_in;
  
  int nErrorsTotal=0;
  int nErrorsConsecutive=0;

	while(1)
  {
    //see if we need to cancel the thread		
    pthread_testcancel();

    //run the update function
		if (di->UpdateFunction() != 0)
    {
      nErrorsTotal++;
      nErrorsConsecutive++;
      if (nErrorsConsecutive >= DEVICE_INTERFACE_MAX_NUM_CONSECUTIVE_ERRORS)
      {
        PRINT_ERROR("**********************************************");        
        PRINT_ERROR("exiting because of too many consecutive errors");
        PRINT_ERROR("**********************************************");
        pthread_exit(NULL);
      }
      usleep(DEVICE_INTERFACE_UPDATE_FUNCTION_ERROR_SLEEP_US);
    }
    else
    {
      //reset the consecutive error count
      nErrorsConsecutive=0;
    }
	}
}

int DeviceInterface::RunSingleUpdate()
{
  if (this->threadRunning)
  {
    PRINT_ERROR("cannot be run if thread is running\n");
    return -1;
  }
  return this->UpdateFunction();
}

int DeviceInterface::UpdateFunction()
{
  double timestamp = 0;

//  bool needToLog = this->GetLoggingEnabled();
  bool needToLog = false;

  if (!this->cBuf)
  {
    PRINT_ERROR("the buffer is NULL\n");
    return -1;
  }

  //get the write pointer
  char * writePtr = NULL;
  if (this->cBuf->GetWritePtr(&writePtr))
  {
    PRINT_ERROR("could not get write pointer\n");
    return -1;
  }
 
  //get data from the device and number of chars read 
  int dataLength=0;
  if ( this->GetData(writePtr, this->cBuf->GetMaxBufferLength(), 
                     dataLength, timestamp, needToLog) )
  {
    PRINT_ERROR("could not get data from device\n");
    if ( this->cBuf->DoneWriting(dataLength) )
    {
      PRINT_ERROR("could not release write pointer\n");
      return -1;
    }
    return -1;
  }
  
  //check the length of data read
  if ( dataLength < 1 )
  {
    PRINT_ERROR("read less than 1 chars from device\n");
    if ( this->cBuf->DoneWriting(dataLength) )
    {
      PRINT_ERROR("could not release write pointer\n");
      return -1;
    }
    return -1;
  }

  //release the write pointer so that the data can be consumed
  if ( this->cBuf->DoneWriting(dataLength,timestamp) )
  {
    PRINT_ERROR("could not release write pointer\n");
    return -1;
  }

  if (needToLog)
  {
//    if (this->FreeLogMem())
//    {
//      PRINT_ERROR("could not free log memory\n");
//      return -1;
//    }
  }
  
  return 0;
}

/*
DATA_LOGGER_TYPE * DeviceInterface::CreateLogger()
{
  this->logFormatter = IPC_parseFormat(this->logFormat.c_str());

  if (IPC_errno == IPC_Illegal_Formatter)
  {
    PRINT_ERROR( "bad ipc data formatter string!!!\n" );
    return NULL;
  }

  return new DATA_LOGGER_TYPE();
}
*/

/*
int DeviceInterface::FreeLogMem()
{
  if (!this->logger)
  {
    PRINT_ERROR("logger is null\n");
    return -1;
  }

  //get a list of QBData that has been logged
  list<QBData> usedData;
  if ( this->logger->GetDone(usedData) )
  {
    PRINT_ERROR("could not get used pointers\n");
    return -1;
  }

  //free the pointers that have been logged
  while(!usedData.empty())
  {
    IPC_freeByteArray(usedData.front().data);
    usedData.pop_front();
    //PRINT_INFO("freed one array\n");
  }
  
  return 0;
}
*/

/*
int DeviceInterface::InitializeLogging(string filename)
{
  if (this->threadRunning)
  {
    PRINT_ERROR("logging can only be enabled when the main thread is not running\n");
    return -1;
  }

  if (this->loggingInitialized)
  {
    PRINT_WARNING("logging already initialized\n");
    return 0;
  }

  if (!this->logger)
  {
    //initialize ipc stuff
    IPC_initialize();

    if (filename.empty())
    {
      PRINT_ERROR("log file name is empty\n");
      return -1;
    }

    this->writeLogFileName = filename;
  
    this->logger = this->CreateLogger();

    if (!this->logger)
    {
      PRINT_ERROR("could not allocate data logger\n");
      return -1;
    }

    if (this->logger->Open(this->writeLogFileName, this->logInfoHeader))
    {
      PRINT_ERROR("could not open log file for writing\n");
      return -1;
    }
  }
  else 
  {
    PRINT_WARNING("logger is already initialized\n");
  }

  this->loggingInitialized = true;
  
  return 0;
}
*/

bool DeviceInterface::IsThreadRunning() { return this->threadRunning; }

int DeviceInterface::EmptyCircularBuffer()
{
  return this->cBuf->EmptyBuffer();
}

int DeviceInterface::GetReadPtr( const char ** dataPtrPtr, int & dataLength, 
                double & timeStamp, double timeoutSec)
{
  return this->cBuf->GetReadPtr(dataPtrPtr,dataLength,timeStamp,timeoutSec);
}

int DeviceInterface::DoneReading( int * numPacketsRemaining)
{
  return this->cBuf->DoneReading(numPacketsRemaining);
}

int DeviceInterface::StartDevice()
{
  return 0;
}

int DeviceInterface::StopDevice()
{
  return 0;
}

int DeviceInterface::LockSettingsMutex()
{
  return pthread_mutex_lock( &this->settingsMutex );
}

int DeviceInterface::UnlockSettingsMutex()
{
  return pthread_mutex_unlock( &this->settingsMutex );
}

//int DeviceInterface::EnableLogging()
//{
//  if (!this->GetLoggingInitialized())
//  {
//    PRINT_ERROR("logger is not initialized\n");
//    return -1;
//  }
//
//  this->LockSettingsMutex();
//  this->loggingEnabled = true;
//  this->UnlockSettingsMutex();
//  return 0;
//}
//
//int DeviceInterface::DisableLogging()
//{
//  this->LockSettingsMutex();
//  this->loggingEnabled = false;
//  this->UnlockSettingsMutex();
//  return 0;
//}
//
//bool DeviceInterface::GetLoggingEnabled()
//{
//  bool enabled;
//  this->LockSettingsMutex();
//  enabled = this->loggingEnabled;
//  this->UnlockSettingsMutex();
//  return enabled;
//}
//
//bool DeviceInterface::GetLoggingInitialized()
//{
//  bool initialized;
//  this->LockSettingsMutex();
//  initialized = this->loggingInitialized;
//  this->UnlockSettingsMutex();
//  return initialized;
//}

int DeviceInterface::SettingsCondSignal()
{
  return pthread_cond_signal(&(this->settingsCond));
}

int DeviceInterface::SettingsCondWait(int timeoutMs)
{ 
  timeval timeStart;
  timespec maxWaitTime;
  gettimeofday(&timeStart,NULL);
  
  maxWaitTime.tv_sec=timeStart.tv_sec + ((timeStart.tv_usec < 1000000-timeoutMs*1000) ? 0:1);
  maxWaitTime.tv_nsec=(timeStart.tv_usec*1000 + timeoutMs*1000000)%1000000000;
  return pthread_cond_timedwait(&(this->settingsCond),&(this->settingsMutex),&maxWaitTime);
}

double DeviceInterface::GetAbsoluteTime()
{
  return Upenn::Timer::GetAbsoluteTime();
}

int DeviceInterface::SetDeviceID(int id)
{
  this->id = id;
  return 0;
}

