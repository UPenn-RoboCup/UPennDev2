
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>
    
#include "dxl_hal.h"

int	  fd	= -1;
long	start_time	= 0;
float	rcv_wait_time	= 0.0f;
float	byte_trans_time	= 0.0f;

char	device_name[20];

int dxl_hal_open( int devIndex, float baudrate )
{

    struct termios options;
    struct serial_struct serinfo;
	  char dev_name[100] = {0, };

    // Make real port name
    sprintf(dev_name, "/dev/ttyUSB%d", devIndex);
    
    printf("Portname: %s\n", dev_name);
    
    strcpy(device_name, dev_name);
	  memset(&options, 0, sizeof(options));
	  dxl_hal_close();
    
    // Open serial device
    if((fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY)) < 0){
      fprintf(stderr, "device open error: %s\n", dev_name);
		  goto DXL_HAL_OPEN_ERROR;
	  }
   
    printf("USB2Dynamixel FD == %X\n", fd);
    
    options.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	  options.c_iflag		= IGNPAR;
	  options.c_oflag		= 0;
	  options.c_lflag		= 0;
	  options.c_cc[VTIME]	= 0;	// time-out 값 (TIME * 0.1초) 0 : disable
	  options.c_cc[VMIN]	= 0;	// MIN 은 read 가 return 되기 위한 최소 문자 개수

	  tcflush(fd, TCIFLUSH);
	  tcsetattr(fd, TCSANOW, &options);
    
    if(fd == -1)
		  return 0;
	
	  if(ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
		  fprintf(stderr, "Cannot get serial info\n");
		  return 0;
	  }
	  
	  serinfo.flags &= ~ASYNC_SPD_MASK;
	  serinfo.flags |= ASYNC_SPD_CUST;
	  serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	  if(ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
		  fprintf(stderr, "Cannot set serial info\n");
		  return 0;
	  }
	
	  dxl_hal_close();
	
	  byte_trans_time = (float)((1000.0f / baudrate) * 12.0f);
	
	  strcpy(device_name, dev_name);
	  memset(&options, 0, sizeof(options));
	  dxl_hal_close();
	
	  if((fd = open(device_name, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		  fprintf(stderr, "device open error: %s\n", dev_name);
		  goto DXL_HAL_OPEN_ERROR;
	  }

	  options.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	  options.c_iflag		= IGNPAR;
	  options.c_oflag		= 0;
	  options.c_lflag		= 0;
	  options.c_cc[VTIME]	= 0;	// time-out 값 (TIME * 0.1초) 0 : disable
	  options.c_cc[VMIN]	= 0;	// MIN 은 read 가 return 되기 위한 최소 문자 개수

	  tcflush(fd, TCIFLUSH);
	  tcsetattr(fd, TCSANOW, &options);
        
    return 1;

DXL_HAL_OPEN_ERROR:
    dxl_hal_close( fd );
    return 0;
}

void dxl_hal_close( )
{
    // Closing device
  if(fd != -1)
		close(fd);
	 fd = -1;
}

int dxl_hal_change_baudrate( float baudrate )
{
   
  struct serial_struct serinfo;
	
	if(fd == -1)
		return 0;
	
	if(ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	dxl_hal_close();
	dxl_hal_open(device_name, baudrate);
	
	byte_trans_time = (float)((1000.0f / baudrate) * 12.0f);
	return 1;
}


void dxl_hal_clear( )
{
    // Clear communication buffer
    tcflush(fd, TCIFLUSH);
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
  // Transmiting date
  //printf("DXL_HAL_TX: transmitted %i bytes.\n", numPacket);
  return write(fd, pPacket, numPacket);
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
    // Recieving date
  //printf("DXL_HAL_RX: received %i bytes.\n", numPacket);

  memset(pPacket, 0, numPacket);
  return read(fd, pPacket, numPacket);
  
}

void  dxl_hal_rx_wait( )
{
    /*
    while(hComm != NULL)
    {
        DWORD dwEvtMask = EV_RXCHAR;
        WaitCommEvent(hComm, &dwEvtMask, NULL);
        if(dwEvtMask & EV_RXCHAR) // Data received
            break;
    }
    */
}

long long dxl_hal_get_perf_counter()
{
    long int counter;
    //QueryPerformanceCounter( &counter );
    //return counter.QuadPart;
    
    return 0;
}

long long dxl_hal_get_perf_freq()
{
    long int freq;
    //QueryPerformanceFrequency( &freq );
    //return freq.QuadPart;

    return 0;
}
