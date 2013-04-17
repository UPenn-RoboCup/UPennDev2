dofile('../include.lua')
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


local skeleton = require 'skeleton'
-- Get the number of users being tracked
local n_users = skeleton.open()
if not n_users then
	print("Cannot open the skeleton!")
	return
end

-- Communications
local unix = require'unix'
local mp = require 'messagepack'
local simple_ipc = require 'simple_ipc'
local sk_pub = simple_ipc.new_publisher('skeleton','position')
local t0 = unix.time()
while true do
	local visible = skeleton.update()
	
	local id = 1
	if visible[2] then
		id = 2;
	end
	if visible[id] then
		local js = ""
		for j=1,15 do
			local position, orientation = skeleton.joint(id,j)
			js = js..mp.pack(joint)
		end
		local nb = sk_pub:send( js )
	end
	
	-- Debug
	local t = unix.time()
	local t_diff = t-t0;
	t0 = t
	if debug then
		print(string.format("%2.2f FPS",1/t_diff))
	end
	
end
-- Shutdown the skeleton
skeleton.shutdown()
