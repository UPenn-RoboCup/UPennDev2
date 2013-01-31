/* 
(c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
University of Pennsylvania
*/

#include "PointerQueueBuffer.hh"
using namespace std;
using namespace Upenn;

PointerQueueBuffer::PointerQueueBuffer(bool keepDone, int maxLength)
{
  this->keepDoneList = keepDone;
  this->maxQueueLength = maxLength;

  pthread_mutex_init( &(this->dataMutex) , NULL );
  pthread_cond_init( &(this->dataCond),NULL);
}

PointerQueueBuffer::~PointerQueueBuffer()
{
  pthread_mutex_destroy(&(this->dataMutex));
  pthread_cond_destroy(&(this->dataCond));
}

int PointerQueueBuffer::Push(QBData * qbd)
{
  this->LockDataMutex();
  int currLength = this->logQueue.size();

  if (currLength >= this->maxQueueLength)
    this->logQueue.pop_front();

  this->logQueue.push_back(*qbd);

  //signal that new data has arrived
  int ret = pthread_cond_signal(&this->dataCond);

  //check the return value
  if (ret)
  {
    PRINT_ERROR("pthread_cond_signal returned unexpected error: "<<ret<<endl);
    this->UnlockDataMutex();
    return -1;
  }

  this->UnlockDataMutex();

  return 0;
}

int PointerQueueBuffer::Pop(QBData * qbd, double timeoutSec)
{
  this->LockDataMutex();
  
  int currLength = this->logQueue.size();

  if (currLength == 0)
  {
    int ret=this->DataCondWait(timeoutSec);
    //check the return value
    if (ret)
    {
      if (ret==ETIMEDOUT)
      {
        //PRINT_WARNING("timeout!\n");
      }
      else
      {
        PRINT_WARNING("unknown error!\n");
      }

      this->UnlockDataMutex();
			return -1;
    }

    //double check to make sure that the queue is still not empty
    currLength = this->logQueue.size();

    if (currLength == 0)
    {
      PRINT_ERROR("CondWait returned 0, but still no data\n");
      this->UnlockDataMutex();
      return -1;
    }
  }

  //copy the data into output pointer
  memcpy(qbd, &(this->logQueue.front()), sizeof(QBData));

  //pop the data that is being returned
  this->logQueue.pop_front();
  
  this->UnlockDataMutex();

  return 0;    
}

int PointerQueueBuffer::PushDone(QBData * qbd)
{
  this->LockDataMutex();
  if (this->keepDoneList)
    this->doneQueue.push_back(*qbd);
  this->UnlockDataMutex();
  return 0;
}

void PointerQueueBuffer::LockDataMutex()
{
  pthread_mutex_lock( &this->dataMutex );
}

void PointerQueueBuffer::UnlockDataMutex()
{
  pthread_mutex_unlock( &this->dataMutex );
}

int PointerQueueBuffer::DataCondWait(double timeoutSec)
{
  if (timeoutSec < 0)
  {
    PRINT_ERROR("timeout must be non-negative\n");
    return -1;
  }
 
	timeval timeStart;
  timespec maxWaitTime;
  gettimeofday(&timeStart,NULL);
  
  int seconds=(int)timeoutSec;
	int uSeconds = 1000000*(timeoutSec-seconds);

  maxWaitTime.tv_sec=timeStart.tv_sec + seconds + ((timeStart.tv_usec < 1000000-uSeconds) ? 0:1);
  maxWaitTime.tv_nsec=(timeStart.tv_usec*1000 + uSeconds*1000)%1000000000;

  //wait until we get a signal or timeout
  return pthread_cond_timedwait(&(this->dataCond), &(this->dataMutex), &maxWaitTime);
}

bool PointerQueueBuffer::IsEmpty()
{
  bool empty;
  this->LockDataMutex();
  empty = this->logQueue.empty();
  this->UnlockDataMutex();

  return empty;
}

int PointerQueueBuffer::GetDone(list<QBData> & done)
{
  done.clear();
  this->LockDataMutex();

  done = this->doneQueue;
  this->doneQueue.clear();

  this->UnlockDataMutex();
  return 0;
}








