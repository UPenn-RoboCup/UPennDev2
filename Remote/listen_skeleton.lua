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
--local sk_pub = simple_ipc.new_publisher('skeleton','position')
local sk_pub = simple_ipc.new_publisher('skeleton')
local t0 = unix.time()
local count = 0;
while true do
	local visible = skeleton.update()
	
	local id = 1
	if visible[2] then
		id = 2;
	end
	if visible[id] then
		local jp = "";
		for j=1,15 do
			local pos = skeleton.joint(id,j)
			jp = jp..mp.pack(pos)
		end
		local nb = sk_pub:send( jp )
	end
	
	-- Debug
	local t = unix.time()
	local t_diff = t-t0
	count=count+1
	if t_diff>1 then
		print(string.format("%2.2f FPS",count/t_diff))
		count = 0
		t0 = t
	end
	
end
-- Shutdown the skeleton
skeleton.shutdown()
