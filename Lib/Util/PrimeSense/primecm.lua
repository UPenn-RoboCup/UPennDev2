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

-- Waist is root...
jointNames = { 
    'Head', 'Neck', 'Torso', 'Waist', -- 1-4
    'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL', --5-10
    'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR', -- 11-16
    'HipL', 'KneeL', 'AnkleL', 'FootL',  -- 17-20
    'HipR', 'KneeR', 'AnkleR', 'FootR' -- 21-24
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

-- Add the joints
shared.joints = {};
shared.joints.qLArm = vector.zeros(3);
shared.joints.qRArm = vector.zeros(3);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

