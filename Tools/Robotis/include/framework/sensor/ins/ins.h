
#ifndef INS_H_
#define INS_H_


#include <pthread.h>
#include <time.h>
#include "../../../../src/framework/sensor/ins/mip_sdk.h"
#include "../../../../src/framework/sensor/ins/byteswap_utilities.h"
#include "motion/motionstatus.h"

namespace Thor
{
	typedef struct 
	{
		double x, y, z;
	}GyroData;

	typedef struct 
	{
		double x,y,z;
	}AccelData;

	class Ins
	{
	private:
		bool navCallBack, ahrsCallBack, gpsCallBack;
		const int DEFAULT_PACKET_TIMEOUT_MS;
		u32 m_bit_result;


		u8  data_stream_format_descriptors[10];
		u16 data_stream_format_decimation[10];
		u8  data_stream_format_num_entries;
		u8  enable;

		u32 nav_valid_packet_count ;
		u32 ahrs_valid_packet_count;
		u32 gps_valid_packet_count ;

		u32 nav_timeout_packet_count  ;
		u32 ahrs_timeout_packet_count ;
		u32 gps_timeout_packet_count  ;

		u32 nav_checksum_error_packet_count  ;
		u32 ahrs_checksum_error_packet_count ;
		u32 gps_checksum_error_packet_count  ;
		mip_interface m_device_interface;

		int m_FBGyroCenterOffset;
		int m_RLGyroCenterOffset;

	public:
		bool m_callbackrunning;
		//AHRS
		mip_ahrs_scaled_gyro  curr_ahrs_gyro;
		mip_ahrs_scaled_accel curr_ahrs_acce;
		mip_ahrs_scaled_mag   curr_ahrs_mag;
		mip_ahrs_euler_angles curr_ahrs_ang;

		mip_ahrs_scaled_gyro  curr_thor_gyro;
		mip_ahrs_scaled_accel curr_thor_accel;
		GyroData strGyroDataforThor;
		AccelData strAccelDataforThor;

		//GPS
		mip_gps_llh_pos curr_llh_pos;
		mip_gps_ned_vel curr_ned_vel;
		mip_gps_time    curr_gps_time;

		//NAV
		mip_nav_llh_pos               curr_nav_pos;
		mip_nav_ned_velocity          curr_nav_vel;
		mip_nav_attitude_euler_angles curr_nav_angles;

		Ins():DEFAULT_PACKET_TIMEOUT_MS(1000), enable(1)
		{ 
			nav_valid_packet_count  = 0;
			ahrs_valid_packet_count = 0;
			gps_valid_packet_count  = 0;

			nav_timeout_packet_count  = 0;
			ahrs_timeout_packet_count = 0;
			gps_timeout_packet_count  = 0;

			nav_checksum_error_packet_count  = 0;
			ahrs_checksum_error_packet_count = 0;
			gps_checksum_error_packet_count  = 0;
			m_callbackrunning = false;
			m_FBGyroCenterOffset = 0;
			m_RLGyroCenterOffset = 0;
			navCallBack = ahrsCallBack = gpsCallBack = false;
		}
		~Ins(){}

		int Connect(const char *com_port, u32 baudrate = 921600);
		void Disconnect();
		mip_interface* GetDeviceInterface();
		int ResetDevice();

		int Initialize();
		bool IsRunning();

		int SetEnableNavDataCallBack();
		int SetEnableAHRSDataCallBack();
		int SetEnableGPSDataCallBack();

		int StartSensingDataUpdate();
		void StopSensingDataUpdate();
		
		int SetGyroOffset(int fb_gyro_offset, int rl_gyro_offset)
		{
			m_FBGyroCenterOffset = fb_gyro_offset;
			m_RLGyroCenterOffset = rl_gyro_offset;
		}

		mip_ahrs_scaled_gyro          GetGyroData() { return curr_ahrs_gyro; }
		mip_ahrs_scaled_accel         GetAccelData() { return curr_ahrs_acce; }
		
		mip_gps_llh_pos               GetGPSData() { return curr_llh_pos; }
		mip_nav_llh_pos               GetLLHPos() { return curr_nav_pos; }
		mip_nav_ned_velocity          GetNedVelocity() { return curr_nav_vel; }
		mip_ahrs_euler_angles         GetEulerAngle()  { return curr_ahrs_ang; }

	protected:
		pthread_t UpdateThread_ID;
		static void *UpdateProc(void *param);
		static void nav_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
		static void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
		static void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
	};
}

#endif
