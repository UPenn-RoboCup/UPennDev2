
#include "CircularBuffer.hh"
#include "ErrorMessage.hh"
#include "Timer.hh"

using namespace std;
using namespace Upenn;

CircularBuffer::CircularBuffer(int newMaxBufferLength, int newNumBuffers)
{
  //error checking
  if (newMaxBufferLength > CIRCULAR_BUFFER_MAX_LENGTH)
  {
    PRINT_ERROR("maximum buffer length exceeds allowed value\n");
    PRINT_ERROR("Requested="<<newMaxBufferLength<<", allowed="<<CIRCULAR_BUFFER_MAX_LENGTH<<"\n");
    exit(1);
  }

  if (newNumBuffers > CIRCULAR_BUFFER_MAX_NUM_BUFFERS)
  {
    PRINT_ERROR("maximum number of buffers exceeds allowed value\n");
    PRINT_ERROR("Requested="<<newNumBuffers<<", allowed="<<CIRCULAR_BUFFER_MAX_NUM_BUFFERS<<"\n");
    exit(1);
  }

  //length of a single buffer
	this->maxBufferLength     = newMaxBufferLength;

  //number of buffers in the ring buffer
  this->numBuffers          = newNumBuffers;

  //allocate memory for the ring buffer
	this->dataBuffers         = new char[this->maxBufferLength * this->numBuffers];
  
  //allocate memory for array, containing number of chars read into a particular buffer
  this->dataBufferLengths   = new int[this->numBuffers];
  
  //memory for keeping track of whether a particular buffer has been read or not
  this->dataBufferFresh     = new int[this->numBuffers];

  //space for time stamps
  this->timeStamps          = new double[this->numBuffers];


  //make sure that all the memory has been successfully allocated
  if ( (this->dataBuffers       == NULL) ||
       (this->dataBufferLengths == NULL) ||
       (this->dataBufferFresh   == NULL) ||
       (this->timeStamps        == NULL) )
  {
    PRINT_ERROR("unable to allocate memory"<<endl);
    exit(1);
  }

  //initialize timestamps to zero
  double t = this->timer0.Tic();
  for (int i=0; i< this->numBuffers; i++)
  {
    this->timeStamps[i] = t;
  }
  
  //zero out variables
  memset(this->dataBuffers,       0, this->maxBufferLength * this->numBuffers);
  memset(this->dataBufferLengths, 0,      this->numBuffers * sizeof(int));
  memset(this->dataBufferFresh,   0,      this->numBuffers * sizeof(int));
  
  //initialize counters
	this->writeCntr  = 0;
	this->readCntr   = 0;
  this->latestCntr =-1;

  //initialize status variables
	this->writing    = false;
	this->reading    = false;

  //init mutexes
	pthread_mutex_init( &this->dataMutex, NULL );
	pthread_cond_init(  &this->dataCond,  NULL );
}

CircularBuffer::~CircularBuffer()
{
	//free up memory
  if ( this->dataBuffers       != NULL)	delete [] this->dataBuffers;
  if ( this->dataBufferLengths != NULL) delete [] this->dataBufferLengths;
  if ( this->dataBufferFresh   != NULL) delete [] this->dataBufferFresh;
  if ( this->timeStamps        != NULL) delete [] this->timeStamps;
  

  //destroy mutexes
	pthread_mutex_destroy( &this->dataMutex );
	pthread_cond_destroy(  &this->dataCond  );
}


void CircularBuffer::LockDataMutex()
{
  pthread_mutex_lock( &this->dataMutex );
}

void CircularBuffer::UnlockDataMutex()
{
  pthread_mutex_unlock( &this->dataMutex );
}

//get a pointer to a buffer for writing
int CircularBuffer::GetWritePtr(char ** dataPtrPtr)
{
	this->LockDataMutex();
  
  if ( this->writing )
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("already requested a write pointer\n");
#endif
    this->UnlockDataMutex();
    return -1;
  }
	
	if ( this->writeCntr == this->readCntr )
  {
		//if the user is actually reading from the buffer, we can't write to it, so jump over
		if ( this->reading )
    {		
			this->writeCntr = (this->writeCntr + 1) % this->numBuffers;
#ifdef CIRCULAR_BUFFER_DEBUG
			PRINT_WARNING("write cntr jumped over read cntr\n");
#endif
		}	
	}

  //write the write pointer to the return variable
	*dataPtrPtr   = (char *) ( this->dataBuffers + (this->writeCntr * this->maxBufferLength) );

	this->writing = true;
	
  this->UnlockDataMutex();
	
  return 0;
}

//call this when done writing to a buffer to make the data available for reading
int CircularBuffer::DoneWriting(int dataLength, double timeStamp)
{
	this->LockDataMutex();

  if ( this->writing )      //make sure that we were writing
  {
    //check if the user wrote too much data (if it's not too late..)
    if ( dataLength > this->maxBufferLength )
    {
#ifdef CIRCULAR_BUFFER_DEBUG
      PRINT_ERROR("wrote too much data: wrote "<<
                 dataLength<<", maxBufferLength="<<this->maxBufferLength<<"\n");
#endif
      this->UnlockDataMutex();
      return -1;
    }
 
    //store the length of the buffer that was read
    this->dataBufferLengths[this->writeCntr]=dataLength;
    if ( dataLength > 0 )
    {
      //mark the buffer as fresh
      this->dataBufferFresh[this->writeCntr] = 1;
      
      //get the unix time stamp
      if (timeStamp < 0 )
        this->timeStamps[this->writeCntr] = this->timer0.GetAbsoluteTime();
      else
        this->timeStamps[this->writeCntr] = timeStamp;
      
      //update the pointer to the latest data
      this->latestCntr=this->writeCntr;

      //update the write counter
      this->writeCntr = (this->writeCntr+1) % this->numBuffers;

      //signal that data is ready
      int ret = pthread_cond_signal(&this->dataCond);

      //check the return value
      if (ret)
      {
#ifdef CIRCULAR_BUFFER_DEBUG
        PRINT_ERROR("pthread_cond_signal returned unexpected error: "<<ret<<endl);
#endif
        this->UnlockDataMutex();
        return -1;
      }
    }
    else 
    {
      this->dataBufferFresh[this->writeCntr] = 0;
    }
    writing = false;
  }
  else
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("not actually writing!!\n");
#endif
    this->UnlockDataMutex();
    exit(1);
  }

	this->UnlockDataMutex();
  return 0;
}

int CircularBuffer::DataCondWait(double timeoutSec)
{
  if (timeoutSec < 0)
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("timeout must be non-negative\n");
#endif
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


//get a pointer from the ring buffer for reading data
//the pointers will be returned in order of filling
//unless you read too slow, then some data will be lost
int CircularBuffer::GetReadPtr(const char ** dataPtrPtr, int & dataLength, double & timeStamp, double timeoutSec)
{
  Timer timer0;
	this->LockDataMutex();

  if (this->reading)
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("already requested a read pointer\n");
#endif
    this->UnlockDataMutex();
    return -1;
  }

	if (this->readCntr == this->writeCntr)
  {
    //timer0.Tic();
    //no data available now - wait for new data or time out
    int ret=this->DataCondWait(timeoutSec);
    double dt = timer0.Toc();
    //printf("waited %f seconds\n",dt);

    //check the return value
    if (ret)
    {
      if (ret==ETIMEDOUT)
      {
#ifdef CIRCULAR_BUFFER_DEBUG
        PRINT_WARNING("timeout!\n");
#endif
      }
      else
      {
#ifdef CIRCULAR_BUFFER_DEBUG
        PRINT_WARNING("unknown error!\n");
#endif
      }

      this->UnlockDataMutex();
			return -1;
    }
    
    //check if the data is available
    if (this->readCntr == this->writeCntr)
    {
#ifdef CIRCULAR_BUFFER_DEBUG
      PRINT_ERROR("this->readCntr=this->writeCntr\n");
#endif
			this->UnlockDataMutex();
			return -1;
    }

    if (this->dataBufferFresh[this->readCntr] == 0)
    {
#ifdef CIRCULAR_BUFFER_DEBUG
      PRINT_ERROR("this->dataBufferFresh[this->readCntr] = 0\n");
#endif
			this->UnlockDataMutex();
			return -1;
    }
	}
	
  //mark the buffer "not fresh"
  this->dataBufferFresh[this->readCntr] = 0;

  //get the pointer to the beginning of the correct buffer
	*dataPtrPtr   = (char *)( this->dataBuffers + (this->readCntr * this->maxBufferLength) );

  //get the number of chars stored in the buffer
  dataLength    = this->dataBufferLengths[this->readCntr];

  //get the time stamp
  timeStamp     = this->timeStamps[this->readCntr];

  //update the read status
	this->reading = true;

	this->UnlockDataMutex();
	
  return 0;
}

int CircularBuffer::GetReadPtrLatest(const char ** dataPtrPtr, int & dataLength, double & timeStamp, double timeoutSec)
{
	this->LockDataMutex();		

  if ( this->reading )
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("already requested a read pointer\n");
#endif
    this->UnlockDataMutex();
    return -1;
  }
    
	if ( (this->latestCntr < 0) || (this->dataBufferFresh[this->latestCntr] == 0) )
  {  
    //no data available now - wait for new data or time out
    int ret = this->DataCondWait(timeoutSec);

    //check the return value
    if (ret)
    {
      if ( ret == ETIMEDOUT )
      {
#ifdef CIRCULAR_BUFFER_DEBUG
        PRINT_WARNING("timeout!\n");
#endif
      }
      else
      {
#ifdef CIRCULAR_BUFFER_DEBUG
        PRINT_WARNING("unknown error in cond wait!\n");
#endif
      }

      this->UnlockDataMutex();
			return -1;
    }

    if ( (this->latestCntr < 0) || (this->dataBufferFresh[this->latestCntr] == 0) )
    {
#ifdef CIRCULAR_BUFFER_DEBUG
      PRINT_ERROR("timeout (2) !\n");
#endif
			this->UnlockDataMutex();
			return -1;
    }
	}

  //mark the buffer "not fresh"
  this->dataBufferFresh[this->readCntr] = 0;
	
  //update the read counter because we are not following the order but reading the latest
  this->readCntr = this->latestCntr;
  
  //get the pointer to the beginning of the correct buffer
	*dataPtrPtr    = (char *)( this->dataBuffers + (this->readCntr * this->maxBufferLength) );
  
  //get the number of chars stored in the buffer
  dataLength     = this->dataBufferLengths[this->readCntr];
 
  //get the time stamp
  timeStamp      = this->timeStamps[this->readCntr];
  
  //update the read status
	this->reading  = true;

	this->UnlockDataMutex();

	return 0;
}

bool CircularBuffer::IsBusy()
{
  bool busy = false;

  this->LockDataMutex();
  if (this->reading)
    busy = true;
  else if (this->writeCntr != this->readCntr)
    busy = true;

  this->UnlockDataMutex();

  return busy;
}

//call this when done reading to make memory available for writing new data
int CircularBuffer::DoneReading(int * numPacketsRemaining)
{
	this->LockDataMutex();
  if ( this->reading )    //make sure that we were reading
  {

    //if the number of remaining packets was requested, fill it in
    if ( numPacketsRemaining != NULL )
    {
      *numPacketsRemaining = this->writeCntr - this->readCntr - 1;
      if ( *numPacketsRemaining < 0 )
        *numPacketsRemaining = *numPacketsRemaining + this->numBuffers;
    }

    //increment the read counter
    if (this->readCntr != this->writeCntr)
      this->readCntr = (this->readCntr + 1) % this->numBuffers;

    //update the reading status
    this->reading  = false;
  }
  else
  {
#ifdef CIRCULAR_BUFFER_DEBUG
    PRINT_ERROR("not actually reading!!\n");
#endif
    this->UnlockDataMutex();
    return -1;
  }

	this->UnlockDataMutex();

	return 0;
}

int CircularBuffer::GetMaxBufferLength()
{ 
  return this->maxBufferLength;
}

int CircularBuffer::GetNumBuffers()
{ 
  return this->numBuffers;
}

double CircularBuffer::GetAbsoluteTime()
{ 
  return this->timer0.GetAbsoluteTime();
}

int CircularBuffer::EmptyBuffer()
{
  this->LockDataMutex();
  while (this->readCntr != this->writeCntr)
  {
    //mark the buffer "not fresh"
    this->dataBufferFresh[this->readCntr] = 0;
    
    this->readCntr = (this->readCntr + 1) % this->numBuffers;
  }

  this->UnlockDataMutex();

  return 0;
}


