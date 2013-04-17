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
#include <map>
#include <vector>
#include <string>
#include <lua.hpp>
#include <OpenNI.h>
#include <NiTE.h>

#ifdef DEBUG
#define USER_MESSAGE(msg) \
{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}
#endif

// Use the right namespace
using namespace openni;

#define MAX_USERS 2 //10 was used before...
std::vector<uint8_t> user_updates;
// Keep track of the skeletons
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
nite::SkeletonJoint g_skeletonJoints[MAX_USERS][NITE_JOINT_COUNT];

// User tracker variables
nite::UserTracker userTracker;
nite::Status niteRc;
nite::UserTrackerFrameRef userTrackerFrame;

// Skeleton Updating function
void updateUserState(const nite::UserData& user, unsigned long long ts)
{
#ifdef DEBUG
	if (user.isNew())
		USER_MESSAGE("New")
			else if (user.isVisible() && !g_visibleUsers[user.getId()])
				USER_MESSAGE("Visible")
					else if (!user.isVisible() && g_visibleUsers[user.getId()])
						USER_MESSAGE("Out of Scene")
							else if (user.isLost())
								USER_MESSAGE("Lost")

									if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
			case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
				break;
			case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
				break;
			case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
				break;
			case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
			case nite::SKELETON_CALIBRATION_ERROR_HANDS:
			case nite::SKELETON_CALIBRATION_ERROR_LEGS:
			case nite::SKELETON_CALIBRATION_ERROR_HEAD:
			case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
				break;
		}
	}
#endif
	int user_id = user.getId();
	g_visibleUsers[user_id] = user.isVisible();
	g_skeletonStates[user_id] = user.getSkeleton().getState();
	for(int jj=0;jj<NITE_JOINT_COUNT;jj++){
		nite::JointType myj = (nite::JointType)jj;
		g_skeletonJoints[user_id][jj] = user.getSkeleton().getJoint(myj);
	}
}

static int lua_skeleton_open(lua_State *L) {
	nite::NiTE::initialize();
	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK) {
		printf("Couldn't create user tracker\n");
		lua_pushnil(L);
		return 1;
	}
	// Push the number of users to be tracked
	lua_pushinteger( L, MAX_USERS );
	return 1;
} //open

static int lua_skeleton_update(lua_State *L) {
	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK)
	{
		printf("Get next frame failed\n");
		lua_pushboolean(L,0);
		return 1;
	}

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];
		updateUserState(user,userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			userTracker.startSkeletonTracking(user.getId());
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

} // update

// Get the whole skeleton
static int lua_retrieve_skeleton(lua_State *L) {
	int user_id = luaL_checkint( L, 1 );
	//nite::Skeleton s = users[user_id].getSkeleton();
	for(int j=0;j<NITE_JOINT_COUNT;j++){
	}
	return 0;
}

// Get a single joint
static int lua_retrieve_joint(lua_State *L) {
	int user_id  = luaL_checkint( L, 1 );
	int joint_id = luaL_checkint( L, 2 );
	
	if( joint_id>NITE_JOINT_COUNT || user_id>MAX_USERS ){
		printf("Out of range! user %d, joint %d\n",user_id,joint_id);
		lua_pushnil(L);
		return 1;
	}
	
	nite::SkeletonJoint j = g_skeletonJoints[user_id-1][joint_id-1];
	lua_newtable(L);
	lua_pushstring(L, "x");   /* Push the table index */
	lua_pushnumber(L,j.getPosition().x); /* Push the cell value */
	lua_rawset(L, -3);      /* Stores the pair in the table */
	lua_pushstring(L, "y");
	lua_pushnumber(L,j.getPosition().y);
	lua_rawset(L, -3);
	lua_pushstring(L, "z");
	lua_pushnumber(L,j.getPosition().z);
	lua_rawset(L, -3);
	lua_pushstring(L, "pc");
	lua_pushnumber(L,j.getPositionConfidence());
	lua_rawset(L, -3);
	// Orientation
	lua_pushnumber(L, j.getOrientationConfidence());
	lua_setfield(L, -2, "oc");

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
	return 1;
}

static int lua_skeleton_shutdown(lua_State *L) {
	nite::NiTE::shutdown();
	printf("Done shutting down the Skeletons\n");
	return 0;
}

static const struct luaL_Reg skeleton_lib [] = {
	{"open", lua_skeleton_open},
	{"update", lua_skeleton_update},
	{"skeleton", lua_retrieve_skeleton},
	{"joint", lua_retrieve_joint},
	{"shutdown", lua_skeleton_shutdown},
	{NULL, NULL}
};

extern "C" int luaopen_skeleton(lua_State *L) {
	luaL_register(L, "skeleton", skeleton_lib);
	return 1;
}