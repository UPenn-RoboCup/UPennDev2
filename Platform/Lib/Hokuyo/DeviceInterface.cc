/* 
(c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
University of Pennsylvania
*/

#include "DeviceInterface.hh"
#include <signal.h>
#include <unistd.h>

using namespace Upenn;

DeviceInterface::DeviceInterface(int bufferLength, int numBuffers)
{
	this->threadRunning           = false;
  this->cBuf                    = new CircularBuffer(bufferLength, numBuffers);
  this->id                      = 0;

  pthread_mutex_init( &(this->settingsMutex) , NULL );
  pthread_cond_init( &(this->settingsCond),NULL);
}

DeviceInterface::~DeviceInterface()
{
  ///stop the thread and shutdown the device
  this->StopThread();
  this->StopDevice();
  
  if (this->cBuf)  delete this->cBuf;

  pthread_mutex_destroy(&(this->settingsMutex));
  pthread_cond_destroy(&(this->settingsCond));
}

//start the main thread
int DeviceInterface::StartThread()
{
  if (this->threadRunning)
  {
    printf("Thread is already running\n");
    return -1;
  }
	
	printf("Starting thread...");

  if (pthread_create(&this->thread, NULL, this->ThreadFunc, (void *)this))
  {
    printf("Could not start thread\n");
    return -1;
  }
  printf("done\n");

	this->threadRunning=true;
	return 0;
}

int DeviceInterface::StopThread()
{
	if (this->threadRunning)
  {
    printf("Stopping thread...");
    pthread_cancel(this->thread);
    pthread_join(this->thread,NULL);
    printf("done\n"); 
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
        printf("**********************************************");        
        printf("exiting because of too many consecutive errors");
        printf("**********************************************");
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
    printf("cannot be run if thread is running\n");
    return -1;
  }
  return this->UpdateFunction();
}

int DeviceInterface::UpdateFunction()
{
  double timestamp = 0;

  if (!this->cBuf)
  {
    printf("the buffer is NULL\n");
    return -1;
  }

  //get the write pointer
  char * writePtr = NULL;
  if (this->cBuf->GetWritePtr(&writePtr))
  {
    printf("could not get write pointer\n");
    return -1;
  }
 
  //get data from the device and number of chars read 
  int dataLength=0;
  if ( this->GetData(writePtr, this->cBuf->GetMaxBufferLength(), 
                     dataLength, timestamp, false) )
  {
    printf("could not get data from device\n");
    if ( this->cBuf->DoneWriting(dataLength) )
    {
      printf("could not release write pointer\n");
      return -1;
    }
    return -1;
  }
  
  //check the length of data read
  if ( dataLength < 1 )
  {
    printf("read less than 1 chars from device\n");
    if ( this->cBuf->DoneWriting(dataLength) )
    {
      printf("could not release write pointer\n");
      return -1;
    }
    return -1;
  }

  //release the write pointer so that the data can be consumed
  if ( this->cBuf->DoneWriting(dataLength,timestamp) )
  {
    printf("could not release write pointer\n");
    return -1;
  }
  
  return 0;
}


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

