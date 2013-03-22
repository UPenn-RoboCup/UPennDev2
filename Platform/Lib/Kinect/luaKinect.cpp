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
#include "luaKinect.hpp"

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

static int lua_kinect_open(lua_State *L) {
  // Should we record? play back? Read from a device?
  const char *file_uri = luaL_optstring(L, 1, NULL);
  const char *logmode = luaL_optstring(L, 2, NULL); // TODO: Add time and date
  if(file_uri==NULL){
    // physical device
    printf("Recording from a physical device...\n");
  } else if( logmode==NULL ){
    // Playback a file
    printf("Playing back log file %s...\n",file_uri);
    //printf("Validity: %d\n",recorder.isValid());
  } else {
    // Log physical device to a file
    printf("Recording a physical device to the logfile %s\n",file_uri);
    recorder.create( file_uri );
    //printf("Validity: %d\n",recorder.isValid());
  }

  // Initialize stream array
  rc = OpenNI::initialize();
  if (rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 0;
  } // if

  if( file_uri!=NULL && logmode==NULL ){
    printf("Opening logfilei...\n");
    rc = device.open( file_uri );
  } else {
    rc = device.open(ANY_DEVICE);
  }
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

  printf("Done initializing the Kinect\n");

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
      for(cntr=0;cntr<NBYTES;cntr++)
        d[cntr] = (uint8_t)( pDepth[cntr]>>3 );
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

  printf("Done shutting down the kinect\n");

  return 0;
}

static const struct luaL_reg kinect_lib [] = {
  {"open", lua_kinect_open},
  {"update", lua_kinect_update},
  {"retrieve", lua_kinect_retrieve},
  {"shutdown", lua_kinect_shutdown},
  {NULL, NULL}
};

int luaopen_Kinect(lua_State *L) {
  luaL_register(L, "Kinect", kinect_lib);
  return 1;
}
