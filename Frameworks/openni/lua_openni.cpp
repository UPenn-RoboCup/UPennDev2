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
#include <string>
#include <OpenNI.h>
#include <NiTE.h>
#include <lua.hpp>
#define MAX_USERS 2 //10 was used before...

char init = 0;

// Use the right namespace
using namespace openni;

/* User Tracking information */
nite::UserTracker *userTracker;
nite::Status niteRc;
// Keep track of the skeletons
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
nite::SkeletonJoint g_skeletonJoints[MAX_USERS][NITE_JOINT_COUNT];
unsigned long long g_skeletonTime[MAX_USERS];

/* Cloud information */
// Establish the streams
VideoStream* m_streams;
VideoStream* depth;
VideoStream* color;

VideoFrameRef* dframe;
VideoFrameRef* cframe;

Device device;
Recorder recorder;
Status rc;
int changedIndex;
uint16_t* pDepth;
uint8_t* pColor;

// Skeleton Updating function
void updateUserState(const nite::UserData& user, unsigned long long ts) {
	int user_id = user.getId();
	g_visibleUsers[user_id] = user.isVisible();
	g_skeletonStates[user_id] = user.getSkeleton().getState();
	g_skeletonTime[user_id] = ts;
	for(int jj=0;jj<NITE_JOINT_COUNT;jj++)
		g_skeletonJoints[user_id][jj] = 
			user.getSkeleton().getJoint((nite::JointType)jj);
}

static int lua_startup(lua_State *L) {
	
	if(init==1)
		return luaL_error(L, "Two OpenNI devices not supported yet.\n" );
	
	/* Initialize the device*/
	// Should we record? play back? Read from a device?
	const char *file_uri = luaL_optstring(L, 1, NULL);
	const char *logmode = luaL_optstring(L, 2, NULL); // TODO: Add time and date
	
#ifdef DEBUG
	if(file_uri==NULL)
		printf("Accessing a physical device...\n");
	else if( logmode==NULL )
		printf("Accessing log file %s...\n",file_uri);
	else
		printf("Recording a physical device to logfile %s\n",file_uri);
#endif
	
	// Setup device
	rc = OpenNI::initialize();
	if (rc != STATUS_OK)
		return luaL_error(L,"OpenNI failed\n%s\n",OpenNI::getExtendedError());
	if( file_uri!=NULL && logmode==NULL )
		rc = device.open( file_uri );
	else
		rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
		return luaL_error(L,"Device failed\n%s\n",OpenNI::getExtendedError());

	m_streams = new VideoStream[2];
	depth = &m_streams[0];
	color = &m_streams[1];
	dframe = new VideoFrameRef;
	cframe = new VideoFrameRef;
	// Setup depth
	if (device.getSensorInfo(SENSOR_DEPTH) == NULL)
		return luaL_error(L,"No depth sensor\n%s\n", OpenNI::getExtendedError());
	rc = depth->create(device, SENSOR_DEPTH);
	if (rc != STATUS_OK)
		return luaL_error(L,"Depth create\n%s\n", OpenNI::getExtendedError());
	rc = depth->start();
	if (rc != STATUS_OK){
		depth->destroy();
		return luaL_error(L,"Depth start\n%s\n", OpenNI::getExtendedError());
	}

	// Setup color
	if (device.getSensorInfo(SENSOR_COLOR) == NULL)
		return luaL_error(L,"No color sensor\n%s\n", OpenNI::getExtendedError());
	rc = color->create(device, SENSOR_COLOR);
	if (rc != STATUS_OK)
		return luaL_error(L,"Depth create\n%s\n", OpenNI::getExtendedError());
	rc = color->start();
	if (rc != STATUS_OK) {
		color->destroy();
		return luaL_error(L,"Color start\n%s\n", OpenNI::getExtendedError());
	}
	
	// Begin recording
	if(file_uri!=NULL && logmode!=NULL )
		recorder.create( file_uri );
	if( recorder.isValid() ){
		recorder.attach( *depth );
		recorder.attach( *color );
		recorder.start();
	}
	
	/* Initialize the Skeleton Tracking system */
	nite::NiTE::initialize();
	userTracker = new nite::UserTracker;
	niteRc = userTracker->create(&device);
	if (niteRc != nite::STATUS_OK)
		return luaL_error(L, "Couldn't create user tracker!\n" );
	
	// Push the number of users to be tracked
	init = 1;
	lua_pushinteger( L, MAX_USERS );
	return 1;
}

static int lua_update_cloud(lua_State *L) {
	rc = OpenNI::waitForAnyStream( &m_streams, 2, &changedIndex);
	if (rc != STATUS_OK)
		return luaL_error(L, "Wait failed!\n" );

	// See which one got updated
	switch (changedIndex) {
		// Depth
		case 0:
		depth->readFrame( dframe );
		pDepth = (uint16_t*)dframe->getData();
		lua_pushnumber( L, 0 );
		lua_pushstring( L, "d" );
		break;
		// Color
		case 1:
		color->readFrame( cframe );
		pColor = (uint8_t*)cframe->getData();
		lua_pushnumber( L, 1 );
		lua_pushstring( L, "c" );
		break;
		// Something went wrong
		default:
		return luaL_error(L, "Error in changedIndex!\n" );
	} // Switch case
	
	// The index and string of the type was pushed
	return 2;
}

static int lua_update_skeleton(lua_State *L) {
	nite::UserTrackerFrameRef userTrackerFrame;
	niteRc = userTracker->readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK)
		return luaL_error(L, "NiTE readFrame failed!\n" );

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];
		updateUserState(user,userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			userTracker->startSkeletonTracking(user.getId());
		}
		else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
		{
#ifdef DEBUG
			const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
			if (head.getPositionConfidence() > .5)
				printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
#endif
		}
	}
	// http://lua-users.org/wiki/SimpleLuaApiExample
	// Begin to return data
	lua_newtable(L);
	for(int u = 1; u <= MAX_USERS; u++) {
		//printf("setting %d\n", (int)g_visibleUsers[u-1] );
		lua_pushnumber(L, u);   /* Push the table index */
		lua_pushboolean(L, g_visibleUsers[u-1] ); /* Push the cell value */
		lua_rawset(L, -3);      /* Stores the pair in the table */
	}
	return 1;

}

// Get a single joint
static int lua_retrieve_joint(lua_State *L) {
	int user_id  = luaL_checkint( L, 1 );
	int joint_id = luaL_checkint( L, 2 );
	
	if( joint_id>NITE_JOINT_COUNT || user_id>MAX_USERS ){
#ifdef DEBUG
		printf("User %d. Joint %d.\n",user_id,joint_id);
#endif
		return luaL_error(L, "Joint or user out of range!\n" );
	}

	// Get the joint
	nite::SkeletonJoint j = g_skeletonJoints[user_id-1][joint_id-1];

	// Position
	lua_newtable(L);
	lua_pushstring(L, "confidence");
	lua_pushnumber(L,j.getPositionConfidence());
	lua_rawset(L, -3);
	lua_pushstring(L, "x");   /* Push the table index */
	lua_pushnumber(L,j.getPosition().x); /* Push the cell value */
	lua_rawset(L, -3);      /* Stores the pair in the table */
	lua_pushstring(L, "y");
	lua_pushnumber(L,j.getPosition().y);
	lua_rawset(L, -3);
	lua_pushstring(L, "z");
	lua_pushnumber(L,j.getPosition().z);
	lua_rawset(L, -3);
	
	// Orientation
	lua_newtable(L);
	lua_pushnumber(L, j.getOrientationConfidence());
	lua_setfield(L, -2, "confidence");
	nite::Quaternion q = j.getOrientation();
	lua_pushstring(L, "quaternion");
	lua_createtable(L, 4, 0);
	lua_pushnumber(L, q.x);
	lua_rawseti(L, -2, 1);
	lua_pushnumber(L, q.y);
	lua_rawseti(L, -2, 2);
	lua_pushnumber(L, q.z);
	lua_rawseti(L, -2, 3);
	lua_pushnumber(L, q.w);
	lua_rawseti(L, -2, 4);
	lua_settable(L, -3);
	
	// Timestamp needed?
	//http://www.lua.org/pil/2.3.html
	lua_pushnumber(L,g_skeletonTime[user_id-1]);
	
	// Returning the Position, Orientation, Timestamp
	return 3;
}

static int lua_retrieve_cloud(lua_State *L) {
	int i = luaL_checkint( L, 1 );
	if(i==0)
		lua_pushlightuserdata( L, pDepth );
	else if(i==1)
		lua_pushlightuserdata( L, pColor );
	else
		return 0;
	return 1;
}

static int lua_skeleton_shutdown(lua_State *L) {
	
	if(init==0)
		return luaL_error(L, "Cannot shutdown an uninitialized object!\n" );
	
	// Kill the recorder if being used
	if( recorder.isValid()==1 ){
		recorder.stop();
		recorder.destroy();
	}

	// Shutoff the streams
	color->stop();
	color->destroy();
	depth->stop();
	depth->destroy();
	device.close();
	
	// Shutdown the NiTE and OpenNI systems
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
#ifdef DEBUG
	printf("Done shutting down NiTE and OpenNI\n");
#endif
	init = 0;
	lua_pushboolean(L,1);
	return 1;
}

static const struct luaL_Reg openni_lib [] = {
	{"startup", lua_startup},
	{"update_skeleton", lua_update_skeleton},
	{"update_cloud", lua_update_cloud},
	{"joint", lua_retrieve_joint},
	{"cloud", lua_retrieve_cloud},
	{"shutdown", lua_skeleton_shutdown},
	{NULL, NULL}
};

extern "C" int luaopen_openni(lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib( L, openni_lib );
#else
	luaL_register(L, "openni", openni_lib);
#endif
	return 1;
}