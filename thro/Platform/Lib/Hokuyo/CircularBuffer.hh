#ifndef CIRCULAR_BUFFER_HH
#define CIRCULAR_BUFFER_HH


#include <pthread.h>
#include "Timer.hh"
#include <signal.h>
#include <errno.h>

#define UPDATE_FUNCTION_ERROR_SLEEP_US 10000
#define MAX_NUM_CONSECUTIVE_ERRORS 10

#define CIRCULAR_BUFFER_MAX_LENGTH 10000000
#define CIRCULAR_BUFFER_MAX_NUM_BUFFERS 10000000

//#define CIRCULAR_BUFFER_DEBUG

namespace Upenn
{

  class CircularBuffer
  {
    //constructor
    public:      CircularBuffer( int newBufferLength, 
                                 int newNumBuffers );
    
    //destructor
	  public:      ~CircularBuffer();

	  public: int   GetWritePtr( char ** dataPtrPtr );

    public: int   DoneWriting( int dataLength, double timeStamp = -1.0);

	  public: int   GetReadPtr( const char ** dataPtrPtr, int & dataLength, 
                              double & timeStamp, double timeoutSec);

    public: int   GetReadPtrLatest( const char ** dataPtrPtr, int & dataLength,
                                    double & timeStamp ,double timeoutSec );

	  public: int   DoneReading( int * numPacketsRemaining = NULL );

    public: int   GetMaxBufferLength();

    public: int   GetNumBuffers();

    public: int   EmptyBuffer();

    public: bool  IsBusy();

    public: double GetAbsoluteTime();

	  private: void LockDataMutex();

	  private: void UnlockDataMutex();

    private: int  DataCondWait( double timeoutSec );

		
		private: pthread_mutex_t  dataMutex;            //mutex for proper reading / writing in threads
		private: pthread_cond_t   dataCond;             //cond variable for waiting with timeout
    private: Timer            timer0;		            //for keeping time
	  private: char           * dataBuffers;          //points to the beginning of the circular buffer
    private: int            * dataBufferLengths;    //stores number of chars read into each buffer
    private: int            * dataBufferFresh;      //status of each buffer (fresh or not)
    private: double         * timeStamps;           //time stamps of the data arrival
	  private: int              maxBufferLength;      //maximum length of a single buffer
	  private: int              numBuffers;           //number of buffers in circular buffer
	  private: int              readCntr;             //index of buffer to read from
	  private: int              writeCntr;            //index of buffer to write to
    private: int              latestCntr;           //index of the latest buffer to read from
	  private: bool             reading;              //is the user reading data now?
	  private: bool             writing;              //is the driver writing data now?
  };
}
#endif //CIRCULAR_BUFFER_HH
