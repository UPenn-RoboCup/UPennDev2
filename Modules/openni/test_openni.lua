local test_cloud = true
local test_skeleton = false
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

if test_skeleton then
	openni.enable_skeleton()
end

local n_users = openni.startup()
if test_skeleton then
	print( "Number of Skeletons:", n_users )
end
local c,d = openni.stream_info()
for k,v in pairs(c) do print(k, v) end
for k,v in pairs(d) do print(k, v) end

-- Assume 30FPS, run for 10 seconds
local nframes = 150;--300
for fr=1,nframes do
	print( string.format("\n======== Frame %d ========",fr) )
	if test_cloud then
		local df, cf = openni.update_rgbd()
		print('Read', type(df), type(cf))
	end

	if test_skeleton then
		visible = openni.update_skeleton()
	else
		visible = false
	end
	if visible then
		for u,v in pairs(visible) do
			if v then
				local pos,orientation,ts = openni.joint(u,test_joint);
				print(string.format('== Timestamp %d ==',ts) )
				if show_position then
					print('==Position==')
					for j,stats in pairs(pos)do
						if type(stats)=="table" then
							print( j, unpack(stats) )
						else
							print( j, stats )
						end
					end
				end -- show position
				if show_orientation then
					print('==Orientation==')
					for j,stats in pairs(orientation)do
						if type(stats)=="table" then
							print( j, unpack(stats) )
						else
							print( j, stats )
						end
					end
				end -- show orientation
			end--v
		end--uv pairs
	end -- visible
end
-- Shutdown the skeleton
print("Shutting down the openni device...")
local shutdown_status = openni.shutdown()
print("Shutdown",shutdown_status)

--[[
local n_users = openni.startup()
print( "Number of Skeletons:", n_users )
local cloud_id, cloud_type = openni.update_cloud()
local visible = openni.update_skeleton()
--]]
print("Shutting down the openni device twice...")
local shutdown_status = openni.shutdown()
print("Shutdown",shutdown_status)
