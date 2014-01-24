/*
 * mjpg_streamer.h
 *
 *  Created on: 2011. 1. 4.
 *      Author: zerom
 */

#ifndef MJPG_STREAMER_H_
#define MJPG_STREAMER_H_

#include <pthread.h>
#include "sensor/vision/image.h"
#include "sensor/vision/camera.h"
#include "httpd.h"

using namespace Thor;

class mjpg_streamer
{
private:
    static globals          global;

    pthread_t               cam;
    pthread_mutex_t         controls_mutex;

    Image*                  input_yuv;
    Image*                  input_rgb;

    context                 server;

    static void* server_thread(void* arg);

public:

    mjpg_streamer(int width, int height);
    mjpg_streamer(int width, int height, char* wwwdir);
    virtual ~mjpg_streamer();

    int input_init();
    int input_cmd(in_cmd_type cmd, int value);
    int send_image(Image* img);

    int output_init();
    int output_run();
};

#endif /* MJPG_STREAMER_H_ */
