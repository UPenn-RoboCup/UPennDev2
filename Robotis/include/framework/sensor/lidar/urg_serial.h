#ifndef URG_SERIAL_H
#define URG_SERIAL_H

/*!
  \file
  \brief ÉVÉäÉAÉãí êM

  \author Satofumi KAMIMURA

  $Id: urg_serial.h,v 1d233c7a2240 2011/02/19 03:08:45 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_detect_os.h"

#if defined(URG_WINDOWS_OS)
#include <windows.h>
#else
#include <termios.h>
#endif
#include "urg_ring_buffer.h"
#include <stdio.h>

enum {
    RING_BUFFER_SIZE_SHIFT = 7,
    RING_BUFFER_SIZE = 1 << RING_BUFFER_SIZE_SHIFT,

    ERROR_MESSAGE_SIZE = 256,
};



typedef struct
{
#if defined(URG_WINDOWS_OS)
    HANDLE hCom;
    int current_timeout;
#else
    int fd;
    struct termios sio;
#endif

    ring_buffer_t ring;
    char buffer[RING_BUFFER_SIZE];
    char has_last_ch;
    char last_ch;
} urg_serial_t;


extern int serial_open(urg_serial_t *serial, const char *device, long baudrate);


extern void serial_close(urg_serial_t *serial);


extern int serial_set_baudrate(urg_serial_t *serial, long baudrate);


extern int serial_write(urg_serial_t *serial, const char *data, int size);


extern int serial_read(urg_serial_t *serial,
                       char *data, int max_size, int timeout);


extern int serial_readline(urg_serial_t *serial,
                           char *data, int max_size, int timeout);


extern int serial_error(urg_serial_t *serial,
                        char *error_message, int max_size);

#ifdef __cplusplus
}
#endif

#endif /* !URG_SERIAL_H */
