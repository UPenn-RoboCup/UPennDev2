module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

--[[
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
--]]

jointNames = { 'Head', 'Neck', 'Torso', 'Waist',
              'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL',
              'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR',              
              'HipL', 'KneeL', 'AnkleL', 'FootL', 
              'HipR', 'KneeR', 'AnkleR', 'FootR',
             };

shared.skeleton = {};
shared.skeleton.found = vector.zeros(1);
shared.skeleton.timestamp = vector.zeros(1);

shared.position = {};
shared.orientation = {};
shared.confidence = {};
for i,v in ipairs(jointNames) do
  shared.position[ v ] = vector.zeros(3);
  shared.orientation[ v ] = vector.zeros(9);
  shared.confidence[ v ] = vector.zeros(2);
end

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

