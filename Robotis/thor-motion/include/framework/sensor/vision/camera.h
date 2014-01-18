/*
 * Camera.h
 *
 *  Created on: 2013. 2. 14.
 *      Author: hjsong
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdlib.h>
#include <linux/videodev2.h>
#include <time.h>

#include "image.h"
#include "framework/minIni.h"

namespace Thor
{
class CameraSettings
{
private:

protected:

public:
    int brightness; /* 0 ~ 255 */
    int contrast;   /* 0 ~ 255 */
    int saturation; /* 0 ~ 255 */
    int gain;       /* 0 ~ 255 */
    int exposure;   /* 0 ~ 10000 */

    CameraSettings() :
        brightness(-1),
        contrast(-1),
        saturation(-1),
        gain(255),
        exposure(1000)
    {}
};


class Camera
{
public:
	static const double VIEW_V_ANGLE = 46.0; //degree
	static const double VIEW_H_ANGLE = 58.0; //degree

    static const int WIDTH  = 320;
    static const int HEIGHT = 240;

private:
       static Camera* uniqueInstance;
       CameraSettings settings;
	   int camera_fd;

	   struct buffer
	   {
        void * start;
        size_t length;
	   };

	   struct buffer* buffers;
	   unsigned int n_buffers;

       Camera();

       void ErrorExit(const char* s);
	   int ReadFrame();

public:
	   bool DEBUG_PRINT;
       FrameBuffer* fbuffer;

       ~Camera();

       static Camera* GetInstance() { return uniqueInstance; }

       int Initialize(int deviceIndex);

	   int v4l2GetControl(int control);
	   int v4l2SetControl(int control, int value);
	   int v4l2ResetControl(int control);

	   void LoadINISettings(minIni* ini);
	   void SaveINISettings(minIni* ini);

	   void SetCameraSettings(const CameraSettings& newset);
	   const CameraSettings& GetCameraSettings();

	   void SetAutoWhiteBalance(int isAuto) { v4l2SetControl(V4L2_CID_AUTO_WHITE_BALANCE, isAuto); }
	   unsigned char GetAutoWhiteBalance() { return (unsigned char)(v4l2GetControl(V4L2_CID_AUTO_WHITE_BALANCE)); }

	   void CaptureFrame();
};

}



#endif /* CAMERA_H_ */
