#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


#ifdef __cplusplus
extern "C" {
#endif



int     dxl_hal_open( int devIndex, float baudrate );
void    dxl_hal_close( );
int     dxl_hal_change_baudrate( float baudrate );
void    dxl_hal_clear( );
int     dxl_hal_tx( unsigned char *pPacket, int numPacket );
int     dxl_hal_rx( unsigned char *pPacket, int numPacket );
void    dxl_hal_rx_wait( );

long long   dxl_hal_get_perf_counter();
long long   dxl_hal_get_perf_freq();



#ifdef __cplusplus
}
#endif

#endif
