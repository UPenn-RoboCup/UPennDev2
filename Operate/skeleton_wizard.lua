dofile'../include.lua'
local openni = require 'openni'
openni.enable_skeleton()
local n_users = openni.startup()
assert(n_users==2,'Should be tracking 2 users maximum')

-- Verify stream
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==320,'Bad depth resolution')
assert(color_info.width==320,'Bad color resolution')

-- How to communicate data?
local use_body = true
local use_zmq = true
local use_udp = false

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

-- Functions for transmitting the data
local function pack_positions(user_id)
	local jp = {}
	for j=1,nJoints do
		jp[j] = openni.joint(user_id,j)
	end
	return mp.pack(jp)
end

local function broadcast_skeleton_zmq()
  local packed_metadata = mp.pack(metadata)
	skeleton_ch1:send({packed_metadata,encoded_joints1})
	skeleton_ch2:send({packed_metadata,encoded_joints2})
end

local function broadcast_skeleton_udp()
	local packed_joints = mp.pack({meta=metadata,raw=encoded_joints1})
	udp_skeleton1:send( packed_joints )
	local packed_joints = mp.pack({meta=metadata,raw=encoded_joints2})
	udp_skeleton2:send( packed_joints )
end

-- Set up the UDP sending
if use_zmq then
  udp = require'udp'
  udp_skeleton1 = udp.new_sender('localhost',43234)
  udp_skeleton2 = udp.new_sender('localhost',43235)
end

-- Set up the ZMQ sending
if use_zmq then
  simple_ipc = require'simple_ipc'
  skeleton_ch1 = simple_ipc.new_publisher('skeleton1')
  skeleton_ch2 = simple_ipc.new_publisher('skeleton2')
end

if use_body then
  Body = require'Body'
end

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
	
  -- Broadcast data
  if use_zmq or use_udp then
    -- Save the metadata
    metadata = {}
    metadata.t = t
    -- Pack the positions data
    enc_jnts1 = nil
    enc_jnts2 = nil
  	if visible[1] then enc_jnts1 = pack_positions(1) end
  	if visible[2] then enc_jnts2 = pack_positions(2) end
    if use_zmq then broadcast_skeleton_zmq(metadata,enc_jnts1,enc_jnts2) end
    if use_udp then broadcast_skeleton_udp(metadata,enc_jnts1,enc_jnts2) end
  end
  
  if use_body then
  end
	
	-- Debug the timing
  cnt = cnt+1
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS.  Tracking", cnt/t_debug)
		if visible[1] then msg = msg..' User 1' end
		if visible[2] then msg = msg..' User 2' end
    --print( 'Torso pos:',unpack( openni.joint(1,NITE_JOINT_TORSO).position ) )
    io.write("Skeleton Wizard | ",msg,'\n\n')
		io.flush()
    t_last = t
    cnt = 0
  end
	
end

-- Shutdown the skeleton
skeleton.shutdown()