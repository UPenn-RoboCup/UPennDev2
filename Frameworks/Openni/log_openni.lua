local test_cloud = true
local test_skeleton = true
local show_position = true
local show_orientation = true

local	NITE_JOINT_HEAD,
NITE_JOINT_NECK,

NITE_JOINT_LEFT_SHOULDER,
NITE_JOINT_RIGHT_SHOULDER,
NITE_JOINT_LEFT_ELBOW,
NITE_JOINT_RIGHT_ELBOW,
NITE_JOINT_LEFT_HAND,
NITE_JOINT_RIGHT_HAND,

NITE_JOINT_TORSO,

NITE_JOINT_LEFT_HIP,
NITE_JOINT_RIGHT_HIP,
NITE_JOINT_LEFT_KNEE,
NITE_JOINT_RIGHT_KNEE,
NITE_JOINT_LEFT_FOOT,
NITE_JOINT_RIGHT_FOOT = 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15

local test_joint = NITE_JOINT_RIGHT_HAND

local openni = require 'openni'
local n_users = openni.startup('/tmp/skel.log','record')
print( "Number of Skeletons:", n_users )

-- Assume 30FPS, run for 10 seconds
local nframes = 300;--300
local cloud_id, cloud_type, visible = nil,nil,nil;
for fr=1,nframes do
	local cloud_id, cloud_type = openni.update_cloud()
	local visible = openni.update_skeleton()
	print(fr)
end
-- Shutdown the skeleton
print("Shutting down the openni device...")
local shutdown_status = openni.shutdown()
print("Shutdown",shutdown_status)
