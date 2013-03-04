#ifndef _DYNAMIXEL_HEADER
#define _DYNAMIXEL_HEADER


#ifdef __cplusplus
extern "C" {
#endif

typedef struct comm{
    int             fd;
    unsigned char   ucReserved;
    int             iBusUsing;
    float           fByteTransTime;
    float           fRcvWaitTime;
    long long       lStartTime;
} SerialPort, *PSerialPort;

typedef struct data {
    int     iStartAddr;
    int     iLength;
    int     iError;
    unsigned char *pucTable;
} BulkData, *PBulkData;

///////////// connection indicator //////////////////////////
#define CONNECT_DEFAULT     (0)
#define CONNECT_LOWSPEED    (1)
#define CONNECT_HIGHSPEED   (2)

////////////////// packet indexes //////////////////////////
enum PACKET_INDEX {
    PKT_HEADER0,
    PKT_HEADER1,
    PKT_HEADER2,
    PKT_RESERVED,
    PKT_ID,
    PKT_LENGTH_L,
    PKT_LENGTH_H,
    PKT_INSTRUCTION,
    PKT_PARAMETER
};

#define BROADCAST_ID        (254)

#define INST_PING           (1)
#define INST_READ           (2)
#define INST_WRITE          (3)
//#define INST_REG_WRITE      (4)
//#define INST_ACTION         (5)
#define INST_FACTORY_RESET  (6)
#define INST_REBOOT			(8)
#define INST_SYSTEM_WRITE	(13)	//0x0D
#define INST_STATUS			(85)	//0x55
#define INST_SYNC_READ		(130)	//0x82
#define INST_SYNC_WRITE     (131)   //0x83
#define INST_BULK_READ      (146)   //0x92
#define INST_BULK_WRITE		(147)	//0x93

#define MAXNUM_TXPARAM      (65535)

#define ERRBIT_VOLTAGE      (1)
#define ERRBIT_ANGLE        (2)
#define ERRBIT_OVERHEAT     (4)
#define ERRBIT_RANGE        (8)
#define ERRBIT_CHECKSUM     (16)
#define ERRBIT_OVERLOAD     (32)
#define ERRBIT_INSTRUCTION  (64)

#define MAXNUM_RXPARAM      (65535)

///////////////// utility for value ///////////////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

////////// packet communication methods ///////////////////////
#define COMM_TXSUCCESS      (0)
#define COMM_RXSUCCESS      (1)
#define COMM_TXFAIL         (2)
#define COMM_RXFAIL         (3)
#define COMM_TXERROR        (4)
#define COMM_RXWAITING      (5)
#define COMM_RXTIMEOUT      (6)
#define COMM_RXCORRUPT      (7)


///////////// device control methods ////////////////////////
int  dxl_initialize( PSerialPort comm, int devIndex, int baudnum );
void dxl_terminate( PSerialPort comm );
int  dxl_change_baudrate( PSerialPort comm, int baudnum );

////////// packet communication methods ///////////////////////
int  dxl_tx_packet( PSerialPort comm, unsigned char *txpacket );
int  dxl_rx_packet( PSerialPort comm, unsigned char *rxpacket );
int  dxl_txrx_packet( PSerialPort comm, unsigned char *txpacket, unsigned char *rxpacket, int *error );

//////////// high communication methods ///////////////////////
int  dxl_ping( PSerialPort comm, int id, int *error );
int  dxl_ping_with_info( PSerialPort comm, int id, int *model_no, int *firm_ver, int *error );
int  dxl_reboot( PSerialPort comm, int id, int *error );
int  dxl_factory_reset( PSerialPort comm, int id, int option, int *error );

int	 dxl_read( PSerialPort comm, int id, int address, int length, unsigned char *data, int *error);
int  dxl_read_byte( PSerialPort comm, int id, int address, int *value, int *error );
int  dxl_read_word( PSerialPort comm, int id, int address, int *value, int *error );
int  dxl_read_dword( PSerialPort comm, int id, int address, unsigned int *value, int *error );

int  dxl_write_byte( PSerialPort comm, int id, int address, int value, int *error );
int  dxl_write_word( PSerialPort comm, int id, int address, int value, int *error );
int  dxl_write_dword( PSerialPort comm, int id, int address, unsigned int value, int *error );

int  dxl_sync_write( PSerialPort comm, int start_addr, int data_length, unsigned char *param, int param_length);
// SyncRead??

int  dxl_bulk_read( PSerialPort comm, unsigned char *param, int param_length, BulkData **rxdata);
int  dxl_get_bulk_byte(PBulkData *data, int id, int addr, int *value);
int  dxl_get_bulk_word(PBulkData *data, int id, int addr, int *value);
int  dxl_get_bulk_dword(PBulkData *data, int id, int addr, unsigned int *value);
// BulkWrite

#ifdef __cplusplus
}
#endif

#endif
