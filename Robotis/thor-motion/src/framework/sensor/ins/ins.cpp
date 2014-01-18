#include "sensor/ins/ins.h"
#include <stdio.h>
#include <time.h>

using namespace Thor;

int Ins::Connect(const char *com_port, u32 baudrate)
{
	int Result = mip_interface_init(atoi(com_port), baudrate, &m_device_interface, DEFAULT_PACKET_TIMEOUT_MS);
	if(Result == MIP_INTERFACE_OK)
		return MIP_INTERFACE_OK;
	else
		return MIP_INTERFACE_ERROR; 
}

void Ins::Disconnect()
{
	mip_interface_close(&m_device_interface);
}

mip_interface* Ins::GetDeviceInterface()
{
	return &m_device_interface;
}

int Ins::Initialize()
{
	int iResult;
	int iMaxAttemptNum = 10;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_base_cmd_idle(&m_device_interface);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
	{
		printf(" fail to idle \n");
		return MIP_INTERFACE_ERROR;
	}

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_base_cmd_ping(&m_device_interface);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
	{
		printf(" fail to ping \n");
		return MIP_INTERFACE_ERROR;
	}

	
	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_base_cmd_built_in_test(&m_device_interface, &m_bit_result);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
	{
		printf(" fail to bit \n");
		return MIP_INTERFACE_ERROR;
	}

	sleep(2);
	
	return MIP_INTERFACE_OK;
}

int Ins::ResetDevice()
{
	int iResult;
	int iMaxAttemptNum = 10;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_base_cmd_reset_device(&m_device_interface);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	return iResult;
}

bool Ins::IsRunning()
{
	return m_callbackrunning;
}


int Ins::SetEnableNavDataCallBack()
{
	data_stream_format_descriptors[0] = MIP_NAV_DATA_LLH_POS; 
	data_stream_format_descriptors[1] = MIP_NAV_DATA_NED_VEL; 
	data_stream_format_descriptors[2] = MIP_NAV_DATA_ATT_EULER_ANGLES;

	data_stream_format_decimation[0]  = 0x01; 
	data_stream_format_decimation[1]  = 0x01; 
	data_stream_format_decimation[2]  = 0x01;

	data_stream_format_num_entries = 3;


	int iResult;
	int iMaxAttemptNum = 10;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_3dm_cmd_nav_message_format(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, 
		data_stream_format_descriptors, data_stream_format_decimation);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
		return iResult;


	
	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_3dm_cmd_continuous_data_stream(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}
	navCallBack = true;
	return iResult;
}

int Ins::SetEnableAHRSDataCallBack()
{
	data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED; 
	data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED; 
	data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;
	data_stream_format_descriptors[3] = MIP_AHRS_DATA_EULER_ANGLES;

	data_stream_format_decimation[0]  = 0x01; 
	data_stream_format_decimation[1]  = 0x01; 
	data_stream_format_decimation[2]  = 0x01;
	data_stream_format_decimation[3]  = 0x01;

	data_stream_format_num_entries = 4;

	int iResult;
	int iMaxAttemptNum = 10;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_3dm_cmd_ahrs_message_format(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, 
		data_stream_format_descriptors, data_stream_format_decimation);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
		return iResult;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_3dm_cmd_continuous_data_stream(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}
	ahrsCallBack = true;
	return iResult;
}

int Ins::SetEnableGPSDataCallBack()
{
	data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS; 
	data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY; 
	data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;

	data_stream_format_decimation[0]  = 0x01; 
	data_stream_format_decimation[1]  = 0x01; 
	data_stream_format_decimation[2]  = 0x01;

	data_stream_format_num_entries = 3;

	int iResult;
	int iMaxAttemptNum = 10;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult = mip_3dm_cmd_gps_message_format(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, 
		data_stream_format_descriptors, data_stream_format_decimation);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}

	if(iResult != MIP_INTERFACE_OK)
		return iResult;

	for(int num = 0; num < iMaxAttemptNum; num++)
	{
		iResult =mip_3dm_cmd_continuous_data_stream(&m_device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM, &enable);
		if(iResult == MIP_INTERFACE_OK)
			break;
	}
	gpsCallBack = true;
	return iResult;
}

int Ins::StartSensingDataUpdate()
{
	if(navCallBack)
		if(mip_interface_add_descriptor_set_callback(&m_device_interface, MIP_NAV_DATA_SET, this, &nav_packet_callback) != MIP_INTERFACE_OK)
			return -1;

	if(ahrsCallBack)
		if(mip_interface_add_descriptor_set_callback(&m_device_interface, MIP_AHRS_DATA_SET, this, &ahrs_packet_callback) != MIP_INTERFACE_OK)
			return -1;

	if(gpsCallBack)
		if(mip_interface_add_descriptor_set_callback(&m_device_interface, MIP_GPS_DATA_SET, this, &gps_packet_callback) != MIP_INTERFACE_OK)
			return -1;


	if(pthread_create(&UpdateThread_ID, NULL, UpdateProc, this) != 0)
		exit(-1);

	m_callbackrunning = true;

	return MIP_INTERFACE_OK;
}

void Ins::StopSensingDataUpdate()
{
	if(m_callbackrunning)
	{
		m_callbackrunning = false;
		if(pthread_join(UpdateThread_ID, 0) != 0)
			exit(-1);
	}
}

void* Ins::UpdateProc(void *param)
{
	Ins *ins = (Ins*)param;
	while(ins->m_callbackrunning)
	{
		mip_interface_update(&(ins->m_device_interface));
		usleep(1);
	}
}

void Ins::nav_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
	Ins *ins = (Ins*)user_ptr;
	mip_field_header *field_header;
	u8               *field_data;
	u16              field_offset = 0;

	//The packet callback can have several types, process them all
	switch(callback_type)
	{
		///
		//Handle valid packets
		///

	case MIP_INTERFACE_CALLBACK_VALID_PACKET:
		{
			ins->nav_valid_packet_count++;

			///
			//Loop through all of the data fields
			///

			while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
			{

				///
				// Decode the field
				///

				switch(field_header->descriptor)
				{
					///
					// Estimated LLH Position
					///

				case MIP_NAV_DATA_LLH_POS:
					{   
						memcpy(&(ins->curr_nav_pos), field_data, sizeof(mip_nav_llh_pos));

						//For little-endian targets, byteswap the data field
						mip_nav_llh_pos_byteswap(&(ins->curr_nav_pos));

					}break;

					///
					// Estimated NED Velocity
					///

				case MIP_NAV_DATA_NED_VEL:
					{
						memcpy(&(ins->curr_nav_vel), field_data, sizeof(mip_nav_ned_velocity));

						//For little-endian targets, byteswap the data field
						mip_nav_ned_velocity_byteswap(&(ins->curr_nav_vel));

					}break;

					///
					// Estimated Attitude, Euler Angles
					///

				case MIP_NAV_DATA_ATT_EULER_ANGLES:
					{
						memcpy(&(ins->curr_nav_angles), field_data, sizeof(mip_nav_attitude_euler_angles));

						//For little-endian targets, byteswap the data field
						mip_nav_attitude_euler_angles_byteswap(&(ins->curr_nav_angles));

					}break;

				default: break;
				}
			} 
		}break;


		///
		//Handle checksum error packets
		///

	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
		{
			ins->nav_checksum_error_packet_count++;
		}break;

		///
		//Handle timeout packets
		///

	case MIP_INTERFACE_CALLBACK_TIMEOUT:
		{
			ins->nav_timeout_packet_count++;
		}break;
	default: break;
	}

}

void Ins::ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
	Ins *ins = (Ins*)user_ptr;
	mip_field_header *field_header;
	u8               *field_data;
	u16              field_offset = 0;

	//The packet callback can have several types, process them all
	switch(callback_type)
	{
		///
		//Handle valid packets
		///

	case MIP_INTERFACE_CALLBACK_VALID_PACKET:
		{
			ins->ahrs_valid_packet_count++;

			///
			//Loop through all of the data fields
			///

			while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
			{

				///
				// Decode the field
				///

				switch(field_header->descriptor)
				{
					///
					// Scaled Accelerometer
					///

				case MIP_AHRS_DATA_ACCEL_SCALED:
					{   
						memcpy(&(ins->curr_ahrs_acce), field_data, sizeof(mip_ahrs_scaled_accel));

						//For little-endian targets, byteswap the data field
						mip_ahrs_scaled_accel_byteswap(&(ins->curr_ahrs_acce));

						ins->curr_thor_accel.scaled_accel[0] =   ins->curr_ahrs_acce.scaled_accel[0];
						ins->curr_thor_accel.scaled_accel[1] = -(ins->curr_ahrs_acce.scaled_accel[1]);
						ins->curr_thor_accel.scaled_accel[2] =   ins->curr_ahrs_acce.scaled_accel[2];
						ins->strAccelDataforThor.x = 128.0 *(ins->curr_thor_accel.scaled_accel[0] + 4.0);
						ins->strAccelDataforThor.y = 128.0 *(ins->curr_thor_accel.scaled_accel[1] + 4.0);
						ins->strAccelDataforThor.z = 128.0 *(ins->curr_thor_accel.scaled_accel[2] + 4.0);

						MotionStatus::FB_ACCEL = ins->strAccelDataforThor.y;
						MotionStatus::RL_ACCEL = ins->strAccelDataforThor.x;

					}break;

					///
					// Scaled Gyro
					///

				case MIP_AHRS_DATA_GYRO_SCALED:
					{
						memcpy(&(ins->curr_ahrs_gyro), field_data, sizeof(mip_ahrs_scaled_gyro));

						//For little-endian targets, byteswap the data field
						mip_ahrs_scaled_gyro_byteswap(&(ins->curr_ahrs_gyro));

						ins->curr_thor_gyro.scaled_gyro[0] = -(ins->curr_ahrs_gyro.scaled_gyro[1]);
						ins->curr_thor_gyro.scaled_gyro[1] = -(ins->curr_ahrs_gyro.scaled_gyro[0]); 
						ins->curr_thor_gyro.scaled_gyro[2] = -(ins->curr_ahrs_gyro.scaled_gyro[2]); 
						ins->strGyroDataforThor.x = 512.0*( (ins->curr_thor_gyro.scaled_gyro[0]))/27.925268;
						ins->strGyroDataforThor.y = 512.0*( (ins->curr_thor_gyro.scaled_gyro[1]))/27.925268;
						ins->strGyroDataforThor.z = 512.0*( (ins->curr_thor_gyro.scaled_gyro[2]))/27.925268;

						MotionStatus::FB_GYRO = ins->strGyroDataforThor.y - (ins->m_FBGyroCenterOffset);
						MotionStatus::RL_GYRO = ins->strGyroDataforThor.x - (ins->m_RLGyroCenterOffset);

					}break;

					///
					// Scaled Magnetometer
					///

				case MIP_AHRS_DATA_MAG_SCALED:
					{
						memcpy(&(ins->curr_ahrs_mag), field_data, sizeof(mip_ahrs_scaled_mag));

						//For little-endian targets, byteswap the data field
						mip_ahrs_scaled_mag_byteswap(&(ins->curr_ahrs_mag));
					}break;
				case MIP_AHRS_DATA_EULER_ANGLES:
					{
						memcpy(&(ins->curr_ahrs_ang), field_data, sizeof(mip_ahrs_euler_angles));
						mip_ahrs_euler_angles_byteswap(&(ins->curr_ahrs_ang));
						MotionStatus::EulerAngleX = (ins->curr_ahrs_ang.pitch);
						MotionStatus::EulerAngleY = (ins->curr_ahrs_ang.roll);
						MotionStatus::EulerAngleZ = -(ins->curr_ahrs_ang.yaw);
					}break;
				default: break;
				}
			} 
		}break;

		///
		//Handle checksum error packets
		///

	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
		{
			ins->ahrs_checksum_error_packet_count++;
		}break;

		///
		//Handle timeout packets
		///

	case MIP_INTERFACE_CALLBACK_TIMEOUT:
		{
			ins->ahrs_timeout_packet_count++;
		}break;
	default: break;
	}


}

void Ins::gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
	Ins *ins = (Ins*)user_ptr;
	mip_field_header *field_header;
	u8               *field_data;
	u16              field_offset = 0;

	//The packet callback can have several types, process them all
	switch(callback_type)
	{
		///
		//Handle valid packets
		///

	case MIP_INTERFACE_CALLBACK_VALID_PACKET:
		{
			ins->gps_valid_packet_count++;

			///
			//Loop through all of the data fields
			///

			while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
			{

				///
				// Decode the field
				///

				switch(field_header->descriptor)
				{
					///
					// LLH Position
					///

				case MIP_GPS_DATA_LLH_POS:
					{   
						memcpy(&(ins->curr_llh_pos), field_data, sizeof(mip_gps_llh_pos));

						//For little-endian targets, byteswap the data field
						mip_gps_llh_pos_byteswap(&(ins->curr_llh_pos));

					}break;

					///
					// NED Velocity
					///

				case MIP_GPS_DATA_NED_VELOCITY:
					{
						memcpy(&(ins->curr_ned_vel), field_data, sizeof(mip_gps_ned_vel));

						//For little-endian targets, byteswap the data field
						mip_gps_ned_vel_byteswap(&(ins->curr_ned_vel));

					}break;

					///
					// GPS Time
					///

				case MIP_GPS_DATA_GPS_TIME:
					{
						memcpy(&(ins->curr_gps_time), field_data, sizeof(mip_gps_time));

						//For little-endian targets, byteswap the data field
						mip_gps_time_byteswap(&(ins->curr_gps_time));

					}break;

				default: break;
				}
			} 
		}break;


		///
		//Handle checksum error packets
		///

	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
		{
			ins->gps_checksum_error_packet_count++;
		}break;

		///
		//Handle timeout packets
		///

	case MIP_INTERFACE_CALLBACK_TIMEOUT:
		{
			ins->gps_timeout_packet_count++;
		}break;
	default: break;
	}


}
