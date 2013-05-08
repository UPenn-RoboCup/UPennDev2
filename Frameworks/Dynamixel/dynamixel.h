/*
C Dynamixel Packet Formation Library (Header)
Daniel D. Lee copyright 2010 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
Yida Zhang copyright 2013 <yida@seas.upenn.edu>
CODE FROM ROBOTIS IS USED IN SELECT PORTIONS
*/

#ifndef __DYNAMIXEL_H
#define __DYNAMIXEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYNAMIXEL_PACKET_HEADER (255) // FF
#define DYNAMIXEL_PACKET_HEADER_2 (255) //FF
#define DYNAMIXEL_PACKET_HEADER_3 (253) //FD
#define DYNAMIXEL_PACKET_STUFFING (0) //XX stuffing
#define DYNAMIXEL_PARAMETER_MAX (250)
#define DYNAMIXEL_BROADCAST_ID (254)

#define INST_PING (1)
#define INST_READ (2)
#define INST_WRITE (3)
	/*
	#define INST_REG_WRITE (4)
	#define INST_ACTION (5)
	*/
#define INST_RESET 	(6)
#define INST_REBOOT (8)
#define INST_SYSTEM_WRITE (13) //0x0D
#define INST_STATUS (85) //0x55
#define INST_SYNC_READ  (130) //0x82
#define INST_SYNC_WRITE (131)
#define INST_BULK_READ  (146) //0x92
#define INST_BULK_WRITE (147) //0x93

#define ERRBIT_VOLTAGE          (1)
#define ERRBIT_ANGLE            (2)
#define ERRBIT_OVERHEAT         (4)
#define ERRBIT_RANGE            (8)
#define ERRBIT_CHECKSUM         (16)
#define ERRBIT_OVERLOAD         (32)
#define ERRBIT_INSTRUCTION      (64)
	
#define MAXNUM_TXPARAM      (65535)
#define MAXNUM_RXPARAM      (65535)

	typedef struct DynamixelPacket {
		uint8_t header1;
		uint8_t header2;
		uint8_t header3;
		uint8_t stuffing;
		uint8_t id;
		uint8_t len[2]; // length does not include first 7 bytes
		uint8_t instruction; // or error for status packets
		uint8_t parameter[MAXNUM_TXPARAM]; // reserve for maximum packet size
		uint16_t checksum; // Needs to be copied at end of parameters
		uint16_t length; // Needs to be copied at end of parameters
	} DynamixelPacket;

	DynamixelPacket *dynamixel_instruction(uint8_t id,
	uint8_t inst,
	uint8_t *parameter,
	uint8_t nparameter);
	DynamixelPacket *dynamixel_instruction_read_data(uint8_t id,
	uint8_t address_h, uint8_t address_l,
	uint8_t n);
	DynamixelPacket *dynamixel_instruction_write_data(uint8_t id,
	uint8_t address_h, uint8_t address_l,
	uint8_t data[], 
	uint8_t n);
	/*
	DynamixelPacket *dynamixel_instruction_reg_write(uint8_t id,
	uint8_t address,
	uint8_t data[], uint8_t n);
	*/
	/*
	DynamixelPacket *dynamixel_instruction_action();
	*/
	DynamixelPacket *dynamixel_instruction_ping(int id);
	DynamixelPacket *dynamixel_instruction_reset(int id);
	DynamixelPacket *dynamixel_instruction_sync_write(
		uint8_t address_h, uint8_t address_l,
	uint16_t len,
	uint8_t data[], uint8_t n);

	//added for bulk read
	DynamixelPacket *dynamixel_instruction_bulk_read_data(
		uint8_t id_cm730, uint8_t id[], uint8_t address, uint8_t len, uint8_t n);
  
	int dynamixel_input(DynamixelPacket *pkt, uint8_t c, int n);

	// Add for Version 2.0 checksum	
	uint16_t dynamixel_crc( uint16_t crc_accum, 
	const unsigned char *data_blk_ptr, 
	uint16_t data_blk_size );
  
#ifdef __cplusplus
}
#endif

#endif // __DYNAMIXEL_H