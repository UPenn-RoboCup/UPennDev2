/*
 * lidar.h
 *
 *  Created on: 2013. 5. 3.
 *      Author: hjsong
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include "framework/motion/motionstatus.h"
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

namespace Thor
{
	class Lidar
	{
	private:
	    urg_t urg;
	    long *data;
	    const char *device;
	    long port;
	    int m_length_data_size;
	    long m_min_distance;
	    long m_max_distance;
	    double m_radian;
	    bool m_callbackrunning;
	    long time_stamp;

	public:
	    Lidar(const char *ip_address);
	    ~Lidar();

	    int open_sensor();
	    int set_scanning_deg(double min_deg, double max_deg);
	    int start_measurement(urg_measurement_type_t type, int scan_times);
	    int get_distance();
	    int get_xy_position();
	    int stop_measurment();

		int StartSensingDataUpdate();
		void StopSensingDataUpdate();

		void close_sensor();

	protected:
		pthread_t UpdateThread_ID;
		static void *UpdateProc(void *param);
	};
}

#endif /* LIDAR_H_ */
