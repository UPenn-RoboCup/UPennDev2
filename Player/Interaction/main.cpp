/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <OpenNI.h>

#include "../Common/OniSampleUtilities.h"

#include <zmq.h>
//#include <zmq.hpp>


using namespace openni;

int main()
{
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}

  // Set up ZMQ
  void *ctx, *socket;
  int result, rc2;
  ctx = zmq_init (1); //1 thread
  assert(ctx);
  socket = zmq_socket (ctx, ZMQ_PUB);
  assert( socket );
  rc2 = zmq_bind( socket, "ipc:///tmp/prime" );
  assert( rc2==0 );
  
  //float xyz[230400];//320*240*3;
  //uint32_t nbytes = 230400*sizeof(float);
  uint8_t d[76800];//[230400];//320*240*3;
  uint32_t nbytes = 76800;
  uint32_t c,cf,i,j;


	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}

	VideoFrameRef frame;

	while (!wasKeyboardHit())
	{
		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Wait failed\n");
			continue;
		}

		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected frame format\n");
			continue;
		}
		//printf("%d x %d\n\n",frame.getHeight(),frame.getWidth() );

		DepthPixel* pDepth = (DepthPixel*)frame.getData();
    c = 0;cf = 0;
    for( j=0; j<240; j++ )
      for( i=0; i<320; i++ ){
        /*
        xyz[c++] = (float)i;
        xyz[c++] = (float)j;
        xyz[c++] = (float)(pDepth[cf++]);
        */
        d[c++] = (uint8_t)( pDepth[cf++]/16 );
      }

    zmq_send( socket, (void*)d, nbytes, 0 );//0flags
    //zmq_send( socket, (void*)xyz, nbytes, 0 );//0flags
    //zmq_send( socket, (void*)pDepth, 2*frame.getHeight()*frame.getWidth(), 0 );//0flags

/*
		int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
		printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);
    */
		int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
		printf("[%08llu] %8d %d\n", (long long)frame.getTimestamp(), pDepth[middleIndex], d[middleIndex]);

	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
