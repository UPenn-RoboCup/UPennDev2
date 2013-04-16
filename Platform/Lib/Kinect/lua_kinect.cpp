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
// LUA WRAPPER (c) Stephen McGill 2013
// 
#include <stdio.h>
#include <vector>
#include <string>
#include <OpenNI2/OpenNI.h>
#include <lua.hpp>

using namespace openni;

#define NBYTES 76800
#define CNBYTES 230400
uint8_t d[NBYTES];
uint8_t c[CNBYTES];
uint32_t cntr;

VideoStream** m_streams;
VideoStream depth, color;
VideoFrameRef frame, cframe;
Device device;
Recorder recorder;
Status rc;
int changedIndex;
uint16_t* pDepth;
uint8_t* pColor;
int lenPacked;
uint8_t zoom = 3;// default zoom

static int lua_kinect_open(lua_State *L) {
	// Should we record? play back? Read from a device?
	const char *file_uri = luaL_optstring(L, 1, NULL);
	const char *logmode = luaL_optstring(L, 2, NULL); // TODO: Add time and date
	if(file_uri==NULL){
		printf("Accessing a physical device...\n");
	} else if( logmode==NULL ){
		printf("Accessing log file %s...\n",file_uri);
	} else {
		printf("Recording a physical device to logfile %s\n",file_uri);
		recorder.create( file_uri );
	}

	// Initialize stream array
	rc = OpenNI::initialize();
	if (rc != STATUS_OK) {
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 0;
	}

	if( file_uri!=NULL && logmode==NULL )
		rc = device.open( file_uri );
	else
		rc = device.open(ANY_DEVICE);

	if (rc != STATUS_OK) {
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 0;
	}

	// Setup the depth
	if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK) {
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 0;
		}
		rc = depth.start();
		if (rc != STATUS_OK) {
			printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
			return 0;
		}
	} // if depth

	// Setup color
	if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
		rc = color.create(device, SENSOR_COLOR);
		if (rc != STATUS_OK) {
			printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
			return 0;
		}
		rc = color.start();
		if (rc != STATUS_OK) {   
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", OpenNI::getExtendedError());
			color.destroy();
			return 0;
		}
	} // if color
	// Establish the variables
	m_streams = new VideoStream*[2];
	m_streams[0] = &depth;
	m_streams[1] = &color;

	printf("Done initializing the Kinect streams\n");

	if( recorder.isValid()==1 ){
		printf("Attaching the recorder... ");
		recorder.attach( depth );
		recorder.attach( color );
		printf("Done! Now starting to record.\n");
		recorder.start();
	}
	// Push a status
	lua_pushinteger( L, 1 );
	return 1;
} //open

//uint16_t* tmp16;
//uint8_t* tmp8, *tmp8a;
uint16_t tmp_d;
static int lua_kinect_update(lua_State *L) {
	rc = OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	if (rc != STATUS_OK) {
		printf("Wait failed\n");
		return 0;
	}

	// See which one got updated
	switch (changedIndex) {
		case 0:
			depth.readFrame(&frame);
			pDepth = (uint16_t*)frame.getData();
			for(cntr=0;cntr<NBYTES;cntr++){
				tmp_d = pDepth[cntr]>>zoom;
				if( tmp_d & 0xFF00 ) // Saturate
					d[cntr] = 0xFF;
				else
					d[cntr] = tmp_d & 0xFF;

				//        tmp8 = (uint8_t*)&(pDepth[cntr]);
				//        tmp8a = tmp8+1;
				//        d[cntr] = ( (*tmp8a<<(8-zoom)) & (0xFF00>>zoom) ) | ( (*tmp8>>zoom)&(0x00FF>>zoom) );
			}
			//      #define IDX 320*60+160
			//      tmp16 = &(pDepth[IDX]);
			//      tmp8 = (uint8_t*)tmp16;
			//      tmp8a = tmp8+1;
			//      printf("%u -> %u (%X) [ %u | %u ]\n",pDepth[IDX],d[IDX], *tmp16, *tmp8, *tmp8a );
			break;
		case 1:
			color.readFrame(&cframe);
			pColor = (uint8_t*)cframe.getData();
			break;
		default:
			printf("Error in wait\n");
	} // Switch case

	// Tell the user what was updated
	lua_pushinteger( L, changedIndex );
	return 1;
} // update

static int lua_kinect_retrieve(lua_State *L) {
	int i = luaL_checkint( L, 1 );
	if(i==0)
		lua_pushlightuserdata( L, d );
	else if(i==1)
		lua_pushlightuserdata( L, pColor );
	else
		return 0;
	return 1;
}

static int lua_set_digital_zoom(lua_State *L) {
	uint8_t tmp = luaL_checkint( L, 1 );
	// 2 through 5 are the best
	if(tmp<2 || tmp>5) // Give the current zoom if out of range
		lua_pushinteger(L, zoom);
	else {
		zoom = tmp;
		lua_pushinteger(L, tmp);
	}
	return 1;
}

static int lua_kinect_shutdown(lua_State *L) {
	if( recorder.isValid()==1 ){
		recorder.stop();
		recorder.destroy();
	}

	color.stop();
	color.destroy();
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	printf("Done shutting down the Kinect\n");

	return 0;
}

static const struct luaL_Reg kinect_lib [] = {
	{"open", lua_kinect_open},
	{"update", lua_kinect_update},
	{"zoom", lua_set_digital_zoom},
	{"retrieve", lua_kinect_retrieve},
	{"shutdown", lua_kinect_shutdown},
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_kinect(lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib( L, kinect_lib );
#else
	luaL_register(L, "kinect", kinect_lib);
#endif
	return 1;
}
