dofile'../include.lua'
local openni = require 'openni'
openni.enable_skeleton()
local n_users = openni.startup()
assert(n_users==2,'Should be tracking 2 users maximum')

-- Verify stream
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==320,'Bad depth resolution')
assert(color_info.width==320,'Bad color resolution')

-- Data packing
local mp = require'msgpack'

-- Useful constants
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
local nJoints = 15

--[[
local function pack_positions(user_id)
	local jp = ''
	for j=1,nJoints do
		local joint = openni.joint(user_id,j)
		jp = jp..mp.pack(joint.position)
	end
	return jp
end
--]]

local function pack_positions(user_id)
	local jp = {}
	for j=1,nJoints do
		jp[j] = openni.joint(user_id,j)
	end
	return mp.pack(jp)
end

-- Set up the UDP sending
local udp = require'udp'
local udp_skeleton1 = udp.new_sender('localhost',43234)
local udp_skeleton2 = udp.new_sender('localhost',43235)

-- Set up the ZMQ sending
local simple_ipc = require'simple_ipc'
skeleton_ch1 = simple_ipc.new_publisher('skeleton1')
skeleton_ch2 = simple_ipc.new_publisher('skeleton2')

-- Set up timing debugging
local cnt = 0;
local t_last = unix.time()
local t_debug = 1

-- Start loop
while true do
	
	-- Update the skeleton
	local visible = openni.update_skeleton()
	
	-- Check the time of acquisition
	local t = unix.time()
	
	-- Acquire and pack the data
	local encoded_joints1 = nil
	local encoded_joints2 = nil
	if visible[1] then
		encoded_joints1 = pack_positions(1)
	end
	if visible[2] then
		encoded_joints2 = pack_positions(2)
	end
	
	-- Save the metadata
	local metadata = {}
	metadata.t = t
	local packed_metadata = mp.pack(metadata)
	
	-- Send Skeleton 1
	if encoded_joints1 then
		-- Send over UDP
		local packed_joints = mp.pack({meta=metadata,raw=encoded_joints1})
		udp_skeleton1:send( packed_joints )
		-- Send over ZMQ
		skeleton_ch1:send({packed_metadata,encoded_joints})
	end
	
	-- Send Skeleton 2
	if encoded_joints2 then
		-- Send over UDP
		local packed_joints = mp.pack({meta=metadata,raw=encoded_joints2})
		udp_skeleton2:send( packed_joints )
		-- Send over ZMQ
		skeleton_ch2:send({packed_metadata,encoded_joints})
	end
	
	-- Debug the timing
  cnt = cnt+1
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS.  Tracking", cnt/t_debug)
		if encoded_joints1 then
			msg = msg..' User 1'
			print( 'Torso position',unpack( openni.joint(1,NITE_JOINT_TORSO).position ) )
		end
		if encoded_joints2 then
			msg = msg..' User 2'
			print( 'Torso position',unpack( openni.joint(2,NITE_JOINT_TORSO).position ) )
		end
    io.write("Skeleton Wizard | ",msg,'\n\n')
		
    t_last = t
    cnt = 0
  end
	
end

-- Shutdown the skeleton
skeleton.shutdown()