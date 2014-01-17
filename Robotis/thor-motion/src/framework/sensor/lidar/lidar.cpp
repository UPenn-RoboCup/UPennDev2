/*
 * lidar.cpp
 *
 *  Created on: 2013. 5. 3.
 *      Author: hjsong
 */

#include "framework/sensor/lidar/lidar.h"

using namespace Thor;

Lidar::Lidar(const char *ip_address) : device(ip_address)
{
    data = 0;
    port = 10940;
    m_length_data_size = 0;
    m_min_distance = 0;
    m_max_distance = 0;
    m_radian = 0.0;
    m_callbackrunning = false;
}

Lidar::~Lidar()
{
	if(data != 0 )
		free(data);
}

int Lidar::open_sensor()
{
	return open_urg_sensor(&urg, device);
}

int Lidar::set_scanning_deg(double min_deg, double max_deg)
{
    return urg_set_scanning_parameter(&urg,
                               urg_deg2step(&urg, min_deg),
                               urg_deg2step(&urg, max_deg), 0);
}

int Lidar::start_measurement(urg_measurement_type_t type, int scan_times)
{
	int result =  urg_start_measurement(&urg, type, scan_times, 0);
	if(result < 0)
		return -1;

	data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
	if (!data) {
		perror("urg_max_index()");
		return -1;
	}
	return 0;
}

int Lidar::get_distance()
{
	int n = 0;
	n = urg_get_distance(&urg, data, &time_stamp);

	if(n <= 0)
		return -1;
	else
	{
		m_length_data_size = n;
		//memcpy( distance , data, m_length_data_size*sizeof(data[0]));
		return 0;
	}
}

int Lidar::get_xy_position()
{

    int i;
    long min_distance;
    long max_distance;

    urg_distance_min_max(&urg, &min_distance, &max_distance);
    for (i = 0; i < m_length_data_size; ++i) {
        long l = data[i];
        double radian;
        long x;
        long y;

        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }
        radian = urg_index2rad(&urg, i);
        x = (long)(l * cos(radian));
        y = (long)(l * sin(radian));

        MotionStatus::lidar_data[0][i] = x;
        MotionStatus::lidar_data[1][i] = y;
    }

    return 0;

}

int Lidar::stop_measurment()
{
	return urg_stop_measurement(&urg);
}

int Lidar::StartSensingDataUpdate()
{
	if(pthread_create(&UpdateThread_ID, NULL, UpdateProc, this) != 0)
		return -1;

	m_callbackrunning = true;
	return 0;
}
void Lidar::StopSensingDataUpdate()
{
	if(m_callbackrunning)
	{
		m_callbackrunning = false;
		if(pthread_join(UpdateThread_ID, 0) != 0)
			exit(-1);
	}
}

void Lidar::close_sensor()
{
	urg_close (&urg);
}

void* Lidar::UpdateProc(void *param)
{
	Lidar *lidar = (Lidar*) param;

	while(lidar->m_callbackrunning)
	{
		lidar->get_distance();
		lidar->get_xy_position();
		usleep(1);
	}
}

