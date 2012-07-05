/****************************************************************************
 *                                                                           *
 *  OpenNI 1.x Alpha                                                         *
 *  Copyright (C) 2011 PrimeSense Ltd.                                       *
 *                                                                           *
 *  This file is part of OpenNI.                                             *
 *                                                                           *
 *  OpenNI is free software: you can redistribute it and/or modify           *
 *  it under the terms of the GNU Lesser General Public License as published *
 *  by the Free Software Foundation, either version 3 of the License, or     *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  OpenNI is distributed in the hope that it will be useful,                *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU Lesser General Public License *
 *  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
 *                                                                           *
 ****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnCppWrapper.h>

#ifdef __cplusplus
extern "C"
{
#endif
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#ifdef __cplusplus
}
#endif

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "./Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define MAX_NUM_USERS 1
#define MAX_NUM_JOINTS 24

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

XnUserID aUsers[MAX_NUM_USERS];
XnUInt16 nUsers;
XnSkeletonJointTransformation tmpJoint;
XnSkeletonJointPosition tmpPosition;
XnUInt32 epochTime = 0;

// New
XnSkeletonJoint aJoints[MAX_NUM_JOINTS];
XnUInt16 nJoints;
XnUInt8 active_users[MAX_NUM_USERS];

// Tables
XnVector3D   posTable[MAX_NUM_USERS][MAX_NUM_JOINTS];
XnConfidence posConfTable[MAX_NUM_USERS][MAX_NUM_JOINTS];
XnMatrix3X3  rotTable[MAX_NUM_USERS][MAX_NUM_JOINTS]; // 9 element float array
XnConfidence rotConfTable[MAX_NUM_USERS][MAX_NUM_JOINTS];

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

XnBool fileExists(const char *fn)
{
  XnBool exists;
  xnOSDoesFileExist(fn, &exists);
  return exists;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d New User %d\n", epochTime, nId);
  // New user found
  if (g_bNeedPose)
  {
    g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
  }
  else
  {
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d Lost user %d\n", epochTime, nId);	
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
  g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
  g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d Calibration started for user %d\n", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  if (eStatus == XN_CALIBRATION_STATUS_OK)
  {
    // Calibration succeeded
    printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);		
    g_UserGenerator.GetSkeletonCap().StartTracking(nId);
  }
  else
  {
    // Calibration failed
    printf("%d Calibration failed for user %d\n", epochTime, nId);
    if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
    {
      printf("Manual abort occured, stop attempting to calibrate!");
      return;
    }
    if (g_bNeedPose)
    {
      g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    }
    else
    {
      g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
  }
}


#define CHECK_RC(nRetVal, what)					    \
  if (nRetVal != XN_STATUS_OK)				    \
{								    \
  printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));    \
  return nRetVal;						    \
}

static int init() {
  XnStatus nRetVal = XN_STATUS_OK;
  xn::EnumerationErrors errors;

  const char *fn = NULL;
  if    (fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
  else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
  else {
    printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
    return XN_STATUS_ERROR;
  }
  printf("Reading config from: '%s'\n", fn);

  nRetVal = g_Context.InitFromXmlFile(fn, g_scriptNode, &errors);
  if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
  {
    XnChar strError[1024];
    errors.ToString(strError, 1024);
    printf("%s\n", strError);
    return (nRetVal);
  }
  else if (nRetVal != XN_STATUS_OK)
  {
    printf("Open failed: %s\n", xnGetStatusString(nRetVal));
    return (nRetVal);
  }

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
  CHECK_RC(nRetVal,"No depth");

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK)
  {
    nRetVal = g_UserGenerator.Create(g_Context);
    CHECK_RC(nRetVal, "Find user generator");
  }

  XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected;
  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
  {
    printf("Supplied user generator doesn't support skeleton\n");
    return 1;
  }
  nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  CHECK_RC(nRetVal, "Register to user callbacks");
  printf("User callbacks\n");  

  nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
  CHECK_RC(nRetVal, "Register to calibration start");
  printf("Starting calibration\n");  
  nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
  CHECK_RC(nRetVal, "Register to calibration complete");
  printf("Completing calibration\n");

  if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
  {
    g_bNeedPose = TRUE;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
    {
      printf("Pose required, but not supported\n");
      return 1;
    }
    nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
    CHECK_RC(nRetVal, "Register to Pose Detected");
    g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
  }

  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  nRetVal = g_Context.StartGeneratingAll();
  CHECK_RC(nRetVal, "StartGenerating");

  printf("Starting to run\n");
  if(g_bNeedPose)
  {
    printf("Assume calibration pose\n");
  }
  /*
     while (!xnOSWasKeyboardHit()){}
     */
}
void close(){
  g_scriptNode.Release();
  g_DepthGenerator.Release();
  g_UserGenerator.Release();
  g_Context.Release();
}

const char *jointToS(XnSkeletonJoint eJoint)
{// http://ndruger.lolipop.jp/hatena/20110108/tcp.diff
	// fool. but can't get the number of XnSkeletonJoint.
	switch (eJoint) {
		case XN_SKEL_HEAD: return "HEAD";
		case XN_SKEL_NECK: return "NECK";
		case XN_SKEL_TORSO: return "TORSO";
		case XN_SKEL_WAIST: return "WAIST";
		case XN_SKEL_LEFT_COLLAR: return "LEFT_COLLAR";
		case XN_SKEL_LEFT_SHOULDER: return "LEFT_SHOULDER";
		case XN_SKEL_LEFT_ELBOW: return "LEFT_ELBOW";
		case XN_SKEL_LEFT_WRIST: return "LEFT_WRIST";
		case XN_SKEL_LEFT_HAND: return "LEFT_HAND";
		case XN_SKEL_LEFT_FINGERTIP: return "LEFT_FINGERTIP";
		case XN_SKEL_RIGHT_COLLAR: return "RIGHT_COLLAR";
		case XN_SKEL_RIGHT_SHOULDER: return "RIGHT_SHOULDER";
		case XN_SKEL_RIGHT_ELBOW: return "RIGHT_ELBOW";
		case XN_SKEL_RIGHT_WRIST: return "RIGHT_WRIST";
		case XN_SKEL_RIGHT_HAND: return "RIGHT_HAND";
		case XN_SKEL_RIGHT_FINGERTIP: return "RIGHT_FINGERTIP";
		case XN_SKEL_LEFT_HIP: return "LEFT_HIP";
		case XN_SKEL_LEFT_KNEE: return "LEFT_KNEE";
		case XN_SKEL_LEFT_ANKLE: return "LEFT_ANKLE";
		case XN_SKEL_LEFT_FOOT: return "LEFT_FOOT";
		case XN_SKEL_RIGHT_HIP: return "RIGHT_HIP";
		case XN_SKEL_RIGHT_KNEE: return "RIGHT_KNEE";
		case XN_SKEL_RIGHT_ANKLE: return "RIGHT_ANKLE";
		case XN_SKEL_RIGHT_FOOT: return "RIGHT_FOOT";
	}

	return "";
}

static int lua_get_jointtables(lua_State *L) {
  int i = luaL_checkint(L, 1)-1;// lua starts with 1
  int joint = luaL_checkint(L, 2);
  if( joint>MAX_NUM_JOINTS || joint<=0 ) {
    lua_pushnil(L);
    return 1;
  }
  joint--; // Lua Input is [1,24], C/C++ is [0,23]

  // Push the position  
  lua_createtable(L, 3, 0);
  lua_pushnumber(L, posTable[i][joint].X);
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, posTable[i][joint].Y);
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, posTable[i][joint].Z);
  lua_rawseti(L, -2, 3);
  //printf("Joint %u\t(%lf,%lf,%lf)\n",joint+1,posTable[joint].X,posTable[joint].Y,posTable[joint].Z);


  // Push the orientation
  lua_createtable(L, 9, 0);
  //printf("Joint %u\t( ",joint+1);  
  for (int i = 0; i < 9; i++) {
    XnFloat* tmp = rotTable[i][joint].elements;
    lua_pushnumber(L, tmp[i]);   /* Push the table index */
    lua_rawseti(L, -2, i+1);
    //printf( "%lf ", tmp[i] );
  }
  //printf(")\n");  

  // Push the confidences
  lua_createtable(L, 2, 0);  
  lua_pushnumber(L, posConfTable[i][joint] );
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, rotConfTable[i][joint] );
  lua_rawseti(L, -2, 2);
  //printf("Joint %u\t(%lf,%lf)\n",joint+1,posConfTable[joint],rotConfTable[joint]);
  
  // Push whether the user is active
  lua_pushnumber(L,active_users[i]);
  return 4;
}

/*
 XnSkeletonJoint { 
  XN_SKEL_HEAD = 1, XN_SKEL_NECK = 2, XN_SKEL_TORSO = 3, XN_SKEL_WAIST = 4, 
  XN_SKEL_LEFT_COLLAR = 5, XN_SKEL_LEFT_SHOULDER = 6, XN_SKEL_LEFT_ELBOW = 7, XN_SKEL_LEFT_WRIST = 8, 
  XN_SKEL_LEFT_HAND = 9, XN_SKEL_LEFT_FINGERTIP = 10, XN_SKEL_RIGHT_COLLAR = 11, XN_SKEL_RIGHT_SHOULDER = 12, 
  XN_SKEL_RIGHT_ELBOW = 13, XN_SKEL_RIGHT_WRIST = 14, XN_SKEL_RIGHT_HAND = 15, XN_SKEL_RIGHT_FINGERTIP = 16, 
  XN_SKEL_LEFT_HIP = 17, XN_SKEL_LEFT_KNEE = 18, XN_SKEL_LEFT_ANKLE = 19, XN_SKEL_LEFT_FOOT = 20, 
  XN_SKEL_RIGHT_HIP = 21, XN_SKEL_RIGHT_KNEE = 22, XN_SKEL_RIGHT_ANKLE = 23, XN_SKEL_RIGHT_FOOT = 24 
}
--OpenNI
XN_SKEL_HEAD 	
XN_SKEL_NECK 	
XN_SKEL_TORSO 	
XN_SKEL_WAIST

XN_SKEL_LEFT_COLLAR 	
XN_SKEL_LEFT_SHOULDER 	
XN_SKEL_LEFT_ELBOW 	
XN_SKEL_LEFT_WRIST 	
XN_SKEL_LEFT_HAND
XN_SKEL_LEFT_FINGERTIP

XN_SKEL_RIGHT_COLLAR 	
XN_SKEL_RIGHT_SHOULDER 	
XN_SKEL_RIGHT_ELBOW 	
XN_SKEL_RIGHT_WRIST 	
XN_SKEL_RIGHT_HAND 	
XN_SKEL_RIGHT_FINGERTIP

XN_SKEL_LEFT_HIP 	
XN_SKEL_LEFT_KNEE 	
XN_SKEL_LEFT_ANKLE 	
XN_SKEL_LEFT_FOOT

XN_SKEL_RIGHT_HIP 	
XN_SKEL_RIGHT_KNEE 	
XN_SKEL_RIGHT_ANKLE 	
XN_SKEL_RIGHT_FOOT
*/

static int lua_update_joints(lua_State *L) {

  g_Context.WaitOneUpdateAll(g_UserGenerator);
  //  printf("Waited one update...\n");
  // print the torso information for the first user already tracking
  nUsers=MAX_NUM_USERS;
  g_UserGenerator.GetUsers(aUsers, nUsers);
  //  printf("Got all users...\n");

  int ret = -1;

  for(XnUInt16 i=0; i<nUsers; i++)
  { 
    if( g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i])==FALSE){
      active_users[i] = 0;
      continue;
    } else {
      // Update which users are active...
      active_users[i] = 1;
      ret = 1;
    }

    // TODO: Error check
    //g_UserGenerator.GetSkeletonCap().EnumerateActiveJoints( aJoints, nJoints );
    //printf("Entering loop with %d joints...\n", nJoints);    
    for( XnUInt16 jj=1; jj<=MAX_NUM_JOINTS; jj++ ){
      XnSkeletonJoint j = XnSkeletonJoint(jj);
      // Use tmpJoint as a temporary joint
      g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(aUsers[i], j, tmpJoint);
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], j, tmpPosition);      
      XnVector3D   tmpPos  = tmpPosition.position;
      XnConfidence posConf = tmpPosition.fConfidence;
      XnMatrix3X3  tmpRot  = tmpJoint.orientation.orientation; // 9 element float array
      XnConfidence rotConf = tmpJoint.orientation.fConfidence;
      //printf("%u (%u): %s \t(%u,%u)\t = %lf:(%lf,%lf,%lf)\n",i, active_users[i],jointToS(j),j,jj,posConf, tmpPos.X,tmpPos.Y,tmpPos.Z);
      // Set the right table value posTable
      posTable[i][jj-1] = tmpPos;
      rotTable[i][jj-1] = tmpRot;
      posConfTable[i][jj-1] = posConf;
      rotConfTable[i][jj-1] = rotConf;
    }
  }
  if(ret==-1){
    lua_pushnil(L);
  } else {
    lua_pushinteger(L, ret);
  }
  return 1;
}

static const struct luaL_reg primesense_lib [] = {
  {"update_joints", lua_update_joints},
  {"get_jointtables", lua_get_jointtables},
  {NULL, NULL}
};

extern "C"
int luaopen_primesense (lua_State *L) {
  luaL_register(L, "primesense", primesense_lib);
  init();

  return 1;
}
