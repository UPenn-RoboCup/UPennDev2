#ifndef POINTER_QUEUE_BUFFER_HH
#define POINTER_QUEUE_BUFFER_HH


#include <pthread.h>
#include "Timer.hh"
#include <signal.h>
#include <errno.h>
#include <list>
#include "ErrorMessage.hh"

#define UPDATE_FUNCTION_ERROR_SLEEP_US 10000
#define MAX_NUM_CONSECUTIVE_ERRORS 10

#define POINTER_QUEUE_BUFFER_DEF_MAX_LENGTH 1000000

using namespace std;

namespace Upenn
{
  //struct for keeping the data in the queue
  struct QBData
  {
    //default constructor
    QBData() : data(NULL), size(0), timestamp(-1.0), info(NULL) {}

    //other constructors
    QBData(char * _data, int _size, double _timestamp)
      : data(_data), size(_size), timestamp(_timestamp), info(NULL) {}

    QBData(char * _data, int _size, double _timestamp, char * _info)
      : data(_data), size(_size), timestamp(_timestamp), info(_info) {}

    //data members
    char * data;
    int size;
    double timestamp;
    char * info;
  };

  //queue buffer class
  class PointerQueueBuffer
  {
    //constructor
    public:       PointerQueueBuffer( bool keepDone = true, 
                     int maxLength = POINTER_QUEUE_BUFFER_DEF_MAX_LENGTH );
    
    //destructor
	  public:      ~PointerQueueBuffer();

    //push the data into the queue
    //public: int   Push(char * dataPtr, int num, double timestamp = -1.0, char * info = NULL);
    public: int Push(QBData * qbd);

    //get the oldest data from the queue
    public: int Pop(QBData * qbd, double timeoutSec);
	  //public: int   Pop(char ** dataPtrPtr, int * num, 
    //                  double * timestamp, double timeoutSec);

    //push the pointer which has been logged into the done queue
    public: int   PushDone(QBData * qbd);

    //check if there is any data in the queue
    public: bool  IsEmpty();

    //get the used pointers so that they can be freed
    public: int GetDone(list<QBData> & doneList);

    //lock the mutex
	  private: void LockDataMutex();

    //unlock the mutex
	  private: void UnlockDataMutex();

    //wait for signal
    private: int  DataCondWait( double timeoutSec );

		
		private: pthread_mutex_t  dataMutex;            //mutex for proper reading / writing in threads
		private: pthread_cond_t   dataCond;             //cond variable for waiting with timeout
    private: bool             keepDoneList;         //do we keep the list of pointers that have been logged
    private: list<QBData>     logQueue;
    private: list<QBData>     doneQueue;
    private: int              maxQueueLength;
  };
}
#endif //POINTER_QUEUE_BUFFER_HH
