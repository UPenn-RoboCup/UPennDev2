#include "CRC16.h"
#include "dxl_hal.h"
#include "dynamixel.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/time.h>

#define DEFAULT_BAUDNUMBER  (1)
#define LATENCY_TIME        (16)    //ms    (USB2Serial Latency timer)

unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;

long	glStartTime	= 0;
float	gfRcvWaitTime	= 0.0f;
float	gfByteTransTime	= 0.0f;

static inline long myclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void dxl_set_timeout(PSerialPort comm, int rcv_length)
{
    // rcv_length: number of recieving data(to calculate maximum waiting time)
    //comm->lStartTime = dxl_hal_get_perf_counter();
    //comm->fRcvWaitTime = (float)(comm->fByteTransTime*(float)rcv_length + 2*LATENCY_TIME + 2.0f);
    
    //glStartTime = myclock();
    comm->lStartTime = myclock();
	  comm->fRcvWaitTime = (float)(gfByteTransTime*(float)rcv_length + 5.0f);
}

int dxl_is_timeout(PSerialPort comm)
{
/*
    long long curr_counter, freq;
    double interval_time;

    curr_counter = dxl_hal_get_perf_counter();
    freq = dxl_hal_get_perf_freq();

    interval_time = (double)(curr_counter - comm->lStartTime) / (double)freq;
    interval_time *= 1000.0;

    if(interval_time > comm->fRcvWaitTime)
        return 1;

    // Overflow
    if(interval_time < 0)
        comm->lStartTime = curr_counter;

    return 0;
*/
  long time;
	
	time = myclock() - comm->lStartTime;
	
	if(time > comm->fRcvWaitTime)
		return 1;
	else if(time < 0)
		comm->lStartTime = myclock();
		
	return 0;
}

float dxl_get_baudrate(int baudnum)
{
    switch(baudnum)
    {
    case 250:
        return 2250000.0f;
    case 251:
        return 2500000.0f;
    case 252:
        return 3000000.0f;
    case 253:
        return 3500000.0f;
    case 254:
        return 4000000.0f;
    case 255:
        return 4500000.0f;
    default:
        return 2000000.0f / (float)(baudnum + 1);
    }    
}

void dxl_add_stuffing(unsigned char * packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    unsigned char temp[MAXNUM_TXPARAM + 10] = {0};

    memcpy(temp, packet, PKT_LENGTH_H+1);    // FF FF FD XX ID LEN_L LEN_H
    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        temp[index++] = packet[i+PKT_INSTRUCTION];
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD
            temp[index++] = 0xFD;
            packet_length_out++;
        }
    }
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    memcpy(packet, temp, index);
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void dxl_remove_stuffing(unsigned char * packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD FD
            packet_length_out--;
            i++;
        }
        packet[index++] = packet[i+PKT_INSTRUCTION];
    }
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

int dxl_initialize( PSerialPort comm, int devIndex, int baudnum )
{
    float baudrate = dxl_get_baudrate(baudnum);
    comm->fByteTransTime = 1000.0f / baudrate * 10.0f; // 1000/baudrate(bit per msec) * 10(start bit + data bit + stop bit)
    if( dxl_hal_open(devIndex, baudrate) == 0 )
        return 0;

    comm->iBusUsing = 0;
    return 1;
}

void dxl_terminate( PSerialPort comm )
{
    dxl_hal_close();
}

int dxl_change_baudrate( PSerialPort comm, int baudnum )
{
    return dxl_hal_change_baudrate(dxl_get_baudrate(baudnum));
}

int dxl_tx_packet( PSerialPort comm, unsigned char *txpacket )
{
    int packet_tx_len, real_tx_len;
    int length, result = COMM_TXFAIL;
    unsigned short crc = 0;

    // Check Bus Using
    if(comm->iBusUsing == 1)
        return COMM_TXFAIL;
    comm->iBusUsing = 1;

    // Character stuffing
    dxl_add_stuffing(txpacket);
    length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]);

    // Check MAX packet length
    if(length > (MAXNUM_TXPARAM+2))
    {
        comm->iBusUsing = 0;
        return COMM_TXERROR;
    }

    // Packet Header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]	= comm->ucReserved;

    // Add CRC16
    crc = update_crc(0, txpacket, length+PKT_LENGTH_H+1-2);  // -2 : except CRC16
    txpacket[length+PKT_LENGTH_H-1] = DXL_LOBYTE(crc);     // last - 1
    txpacket[length+PKT_LENGTH_H-0] = DXL_HIBYTE(crc);     // last - 0

    // Tx Packet
    dxl_hal_clear();
    packet_tx_len = length + PKT_LENGTH_H + 1;
    real_tx_len = dxl_hal_tx( txpacket, packet_tx_len );
    if( packet_tx_len != real_tx_len )
    {
        comm->iBusUsing = 0;
        return COMM_TXFAIL;
    }

    return COMM_TXSUCCESS;
}

int dxl_rx_packet( PSerialPort comm, unsigned char *rxpacket )
{
    int rx_length = 0, wait_length = PKT_LENGTH_H + 4 + 1;    // 4 : INST ERROR CHKSUM_L CHKSUM_H
    int i, j, result = COMM_RXFAIL;
    unsigned short crc = 0;

    // Check Bus Using
    //if(bus_using == 0)
    //    return 0;

    while(1)
    {
        rx_length += dxl_hal_rx( &rxpacket[rx_length], wait_length - rx_length);
        if(rx_length >= wait_length)    // wait_length minimum : 11
        {
            // Find packet header
            for(i = 0; i < (rx_length - 2); i++)
            {
                if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF && rxpacket[i+2] == 0xFD)
                    break;
            }

            if(i == 0)
            {
                // Check length
                wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
                if(rx_length < wait_length)
                {
                    // Check timeout
                    if(dxl_is_timeout(comm) == 1)
                    {
                        if(rx_length == 0)
                            result = COMM_RXTIMEOUT;
                        else
                            result = COMM_RXCORRUPT;
                        comm->iBusUsing = 0;
                        break;
                    }
                    continue;
                }

                // Check CRC16
                crc = DXL_MAKEWORD(rxpacket[rx_length-2], rxpacket[rx_length-1]);
                if(update_crc(0, rxpacket, wait_length-2) == crc) // -2 : except CRC16
                    result = COMM_RXSUCCESS;
                else
                    result = COMM_RXCORRUPT;
                comm->iBusUsing = 0;
                break;
            }
            else
            {
                // Remove unnecessary packets
                for(j = 0; j < (rx_length - i); j++)
                    rxpacket[j] = rxpacket[j+i];
                rx_length -= i;
            }
        }
        else
        {
            // Check timeout
            if(dxl_is_timeout(comm) == 1)
            {
                if(rx_length == 0)
                    result = COMM_RXTIMEOUT;
                else
                    result = COMM_RXCORRUPT;
                comm->iBusUsing = 0;
                break;
            }
        }
    }

    // Character stuffing
    if(result == COMM_RXSUCCESS)
        dxl_remove_stuffing(rxpacket);

    comm->iBusUsing = 0;
    return result;
}

int dxl_txrx_packet( PSerialPort comm, unsigned char *txpacket, unsigned char* rxpacket, int *error )
{
    int result = COMM_TXFAIL;

    // Wait for Bus Idle
    while(comm->iBusUsing == 1) 
    {
        //Sleep(0);
    }

    result = dxl_tx_packet(comm, txpacket);

    // Check Tx packet result
    if( result != COMM_TXSUCCESS )
        return result;

    // Set Rx Timeout
    if(txpacket[PKT_INSTRUCTION] == INST_READ)
        dxl_set_timeout(comm, DXL_MAKEWORD(txpacket[PKT_PARAMETER+2], txpacket[PKT_PARAMETER+3]) + 11);
    else
        dxl_set_timeout(comm, PKT_LENGTH_H+4+1);    // 4 : INST ERROR CHKSUM_L CHKSUM_H

    // BroadCast ID && !BulkRead = There is no need to wait for a rxpacket
    if(txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ)
    {
        comm->iBusUsing = 0;
        return COMM_RXSUCCESS;
    }

    result = dxl_rx_packet(comm, rxpacket);

    if(result == COMM_RXSUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }

    return result;
}

int dxl_ping( PSerialPort comm, int id, int *error)
{
	return dxl_ping_with_info(comm, id, 0, 0, error);
}

int dxl_ping_with_info( PSerialPort comm, int id, int *model_no, int *firm_ver, int *error )
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[10]	= {0};
    unsigned char rxpacket[14]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x03;
    txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_PING;

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);

    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
		if(model_no != 0)
			*model_no = DXL_MAKEWORD(rxpacket[PKT_PARAMETER+1], rxpacket[PKT_PARAMETER+2]);
		if(firm_ver != 0)
			*firm_ver = rxpacket[PKT_PARAMETER+3];
    }
    return result;
}

int dxl_reboot( PSerialPort comm, int id, int *error)
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[10]	= {0};
    unsigned char rxpacket[11]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x03;
    txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_REBOOT;

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);

    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }
    return result;
}

int dxl_factory_reset( PSerialPort comm, int id, int option, int *error )
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[11]	= {0};
    unsigned char rxpacket[11]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x04;
    txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;
	txpacket[PKT_PARAMETER]		= (unsigned char)option;

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);

    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }
    return result;
}

int dxl_read( PSerialPort comm, int id, int address, int length, unsigned char* data, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char txpacket[14]	= {0};
	unsigned char* rxpacket		= (unsigned char*)calloc(length+11, sizeof(unsigned char));

  txpacket[PKT_ID]            = (unsigned char)id;
  txpacket[PKT_LENGTH_L]      = 0x07;
  txpacket[PKT_LENGTH_H]      = 0x00;
  txpacket[PKT_INSTRUCTION]   = INST_READ;
  txpacket[PKT_PARAMETER]     = (unsigned char)DXL_LOBYTE(address);
  txpacket[PKT_PARAMETER+1]   = (unsigned char)DXL_HIBYTE(address);
  txpacket[PKT_PARAMETER+2]   = (unsigned char)DXL_LOBYTE(length);
  txpacket[PKT_PARAMETER+3]   = (unsigned char)DXL_HIBYTE(length);

  result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);
  
  if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
  {
    if(error != 0)
        *error = (int)rxpacket[PKT_PARAMETER];

	  rxpacket += PKT_PARAMETER + 1;
	  memcpy(data, rxpacket, length);
	  rxpacket -= PKT_PARAMETER + 1;
	  
	  // MOVED THIS UP HERE. IT CAUSES MEMORY LEAKS BUT DOESNT CRASH
	  free(rxpacket);
  }

  // COMMENTED THIS OUT. IT CAUSES ERRORS
  //free(rxpacket);
  return result;
}

int dxl_read_byte( PSerialPort comm, int id, int address, int *value, int *error )
{
	int result = COMM_TXFAIL;
	unsigned char data[1] = {0};

	result = dxl_read(comm, id, address, 1, data, error);
	if(result == COMM_RXSUCCESS)
		*value = data[0];

  return result;
}

int dxl_read_word( PSerialPort comm, int id, int address, int *value, int *error )
{
	int result = COMM_TXFAIL;
	unsigned char data[2] = {0};

	result = dxl_read(comm, id, address, 2, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEWORD(data[0], data[1]);

    return result;
}

int dxl_read_dword( PSerialPort comm, int id, int address, unsigned int *value, int *error )
{
	int result = COMM_TXFAIL;
	unsigned char data[4] = {0};

	result = dxl_read(comm, id, address, 4, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]), DXL_MAKEWORD(data[2], data[3]));
  else
    printf("ERROR: dxl_read_dword\n");

    return result;
}

int dxl_write_byte( PSerialPort comm, int id, int address, int value, int *error )
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[13]	= {0};
    unsigned char rxpacket[11]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x06;
    txpacket[PKT_LENGTH_H]      = 0x00;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER]     = (unsigned char)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER+1]   = (unsigned char)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER+2]   = (unsigned char)value;

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);
    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }

    return result;
}

int dxl_write_word( PSerialPort comm, int id, int address, int value, int *error )
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[14]	= {0};
    unsigned char rxpacket[11]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x07;
    txpacket[PKT_LENGTH_H]      = 0x00;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER]     = (unsigned char)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER+1]   = (unsigned char)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER+2]   = (unsigned char)DXL_LOBYTE(value);
    txpacket[PKT_PARAMETER+3]   = (unsigned char)DXL_HIBYTE(value);

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);
    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }

    return result;
}

int dxl_write_dword( PSerialPort comm, int id, int address, unsigned int value, int *error )
{
    int result = COMM_TXFAIL;
    unsigned char txpacket[16]	= {0};
    unsigned char rxpacket[11]	= {0};

    txpacket[PKT_ID]            = (unsigned char)id;
    txpacket[PKT_LENGTH_L]      = 0x09;
    txpacket[PKT_LENGTH_H]      = 0x00;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER]     = (unsigned char)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER+1]   = (unsigned char)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER+2]   = (unsigned char)DXL_LOBYTE(DXL_LOWORD(value));
    txpacket[PKT_PARAMETER+3]   = (unsigned char)DXL_HIBYTE(DXL_LOWORD(value));
    txpacket[PKT_PARAMETER+4]   = (unsigned char)DXL_LOBYTE(DXL_HIWORD(value));
    txpacket[PKT_PARAMETER+5]   = (unsigned char)DXL_HIBYTE(DXL_HIWORD(value));

    result = dxl_txrx_packet(comm, txpacket, rxpacket, 0);
    if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
    {
        if(error != 0)
            *error = (int)rxpacket[PKT_PARAMETER];
    }

    return result;
}

int dxl_sync_write( PSerialPort comm, int start_addr, int data_length, unsigned char *param, int param_length)
{
    int result = COMM_TXFAIL, n;
    int pkt_length = param_length + 7;  // 7 : INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CHKSUM_L CHKSUM_H
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0};

    txpacket[PKT_ID]            = (unsigned char)BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = (unsigned char)DXL_LOBYTE(pkt_length);
    txpacket[PKT_LENGTH_H]      = (unsigned char)DXL_HIBYTE(pkt_length);
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER]     = (unsigned char)DXL_LOBYTE(start_addr);
    txpacket[PKT_PARAMETER+1]   = (unsigned char)DXL_HIBYTE(start_addr);
    txpacket[PKT_PARAMETER+2]   = (unsigned char)DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER+3]   = (unsigned char)DXL_HIBYTE(data_length);
    for(n = 0; n < param_length; n++)
        txpacket[PKT_PARAMETER+4+n] = param[n];

    return dxl_txrx_packet(comm, txpacket, rxpacket, 0);
}

int dxl_bulk_read( PSerialPort comm, unsigned char *param, int param_length, PBulkData *rxdata)
{
    int result = COMM_TXFAIL, n, wait_length = 0;
    int num = param_length / 5; // each length : 5 (ID ADDR_L ADDR_H LEN_L LEN_H)
    int pkt_length = param_length + 3;  // 3 : INST CHKSUM_L CHKSUM_H
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0};

    txpacket[PKT_ID]            = (unsigned char)BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = (unsigned char)DXL_LOBYTE(pkt_length);
    txpacket[PKT_LENGTH_H]      = (unsigned char)DXL_HIBYTE(pkt_length);
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
    for(n = 0; n < param_length; n++)
        txpacket[PKT_PARAMETER+n] = param[n];

    for(n = 0; n < num; n++)
    {
        int id = param[n*5+0];
        rxdata[id]->iError = -1;
        rxdata[id]->iStartAddr = DXL_MAKEWORD(param[n*5+1], param[n*5+2]);
        rxdata[id]->iLength = DXL_MAKEWORD(param[n*5+3], param[n*5+4]);
        rxdata[id]->pucTable = (unsigned char *)calloc(rxdata[id]->iLength, sizeof(unsigned char));
        wait_length += rxdata[id]->iLength + 11;
    }

    /************ TxRxPacket *************/
    // Wait for Bus Idle
    while(comm->iBusUsing == 1) 
    {
        //Sleep(0);
    }

    result = dxl_tx_packet(comm, txpacket);

    // Check Tx packet result
    if( result != COMM_TXSUCCESS )
        return result;

    // Set Rx Timeout (BULK_READ)
    dxl_set_timeout(comm, wait_length);

    for(n = 0; n < num; n++)
    {
        int id = param[n*5+0];
        // Rx packet
        result = dxl_rx_packet(comm, rxpacket);
        if(result == COMM_RXSUCCESS)
            rxdata[id]->iError = rxpacket[PKT_PARAMETER];
        // rxpacket to rxdata[id]->pucTable
        memcpy(rxdata[id]->pucTable, &rxpacket[PKT_PARAMETER+1], rxdata[id]->iLength);
    }


    return result;
}

int dxl_get_bulk_byte(PBulkData *data, int id, int addr, int *value)
{
    if(data[id] == 0 || data[id]->iError == -1)
        return 0;
    if(addr < data[id]->iStartAddr || data[id]->iStartAddr+data[id]->iLength-1 < addr)
        return 0;
    *value = data[id]->pucTable[(addr-data[id]->iStartAddr)];
    return 1;
}

int dxl_get_bulk_word(PBulkData *data, int id, int addr, int *value)
{
    if(data[id] == 0 || data[id]->iError == -1)
        return 0;
    if(addr < data[id]->iStartAddr || data[id]->iStartAddr+data[id]->iLength-2 < addr)  // 2byte
        return 0;
    *value = DXL_MAKEWORD(data[id]->pucTable[(addr-data[id]->iStartAddr)],
                          data[id]->pucTable[(addr-data[id]->iStartAddr+1)]);
    return 1;
}

int dxl_get_bulk_dword(PBulkData *data, int id, int addr, unsigned int *value)
{
    if(data[id] == 0 || data[id]->iError == -1)
        return 0;
    if(addr < data[id]->iStartAddr || data[id]->iStartAddr+data[id]->iLength-4 < addr)  // 4byte
        return 0;
    *value = DXL_MAKEDWORD(DXL_MAKEWORD(data[id]->pucTable[(addr-data[id]->iStartAddr)],
                                        data[id]->pucTable[(addr-data[id]->iStartAddr+1)]),
                           DXL_MAKEWORD(data[id]->pucTable[(addr-data[id]->iStartAddr+2)],
                                        data[id]->pucTable[(addr-data[id]->iStartAddr+3)]));
    return 1;
}
