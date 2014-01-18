/*
 * imgprocess.h
 *
 *  Created on: 2013. 2. 14.
 *      Author: hjsong
 */

#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_

#include "sensor/vision/image.h"

namespace Thor
{
	class ImgProcess
	{
	public:
		static void YUVtoRGB(FrameBuffer *buf);
		static void RGBtoHSV(FrameBuffer *buf);

		static void Erosion(Image* img);
        static void Erosion(Image* src, Image* dest);
		static void Dilation(Image* img);
        static void Dilation(Image* src, Image* dest);

        static void HFlipYUV(Image* img);
        static void VFlipYUV(Image* img);
	};
}


#endif /* IMGPROCESS_H_ */
