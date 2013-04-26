dofile('../include.lua')
local unix = require 'unix'

-- OpenNI Convention
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

-- Grabbing device
local openni = require 'openni'
assert(openni.startup('/tmp/skel.log')==2,'bad init of device')

-- Sending Data
local mp = require 'msgpack'
local simple_ipc = require 'simple_ipc'
local sk_ch = simple_ipc.new_publisher('skeleton')
local ac_ch = simple_ipc.new_publisher('actuator_cmd')

-- Save image
local cjpeg = require 'cjpeg'

-- CHARLI
require 'Params'
local Transform = require 'Transform'
local K = require 'Kinematics'
local carray = require 'carray'
local nJoint = 23;
local wbJoints = carray.double( nJoint )

-- logs have 300 frames
local nframes = 300;
local fr_a,fr_b=110,300
--local fr_a,fr_b=110,200
local cloud_id, cloud_type, visible = nil,nil,nil;

function run()
	for fr=1,nframes do
		local cloud_id, cloud_type = openni.update_cloud()
		local visible = openni.update_skeleton()
		for u,v in pairs(visible) do
			if v and fr>=fr_a and fr<=fr_b then
				local mp_sk = '';
				print( string.format('Progress %d',fr) )
				local pos_tbl = {}
				for j=1,15 do
					local pos,orientation,ts = openni.joint( u, j );
					--local pos2 = vector.new({pos.x,pos.y,pos.z})
					--local pos2 = vector.new({pos.z,pos.x,pos.y})
					local pos2 = vector.new({pos.z,-1*pos.x,pos.y})
					pos_tbl[j] = pos2
					local mp_pos = mp.pack(pos)
					mp_sk = mp_sk..mp_pos;
				end
				-- Send mp data to MATLAB
				assert( sk_ch:send(mp_sk) )
				-- Run IK and send to Webots
				ik(pos_tbl)
				assert( ac_ch:send(wbJoints:string()) )
			end
		end
	end
end

function ik( sk_pos )

	-- Adjust pose to the torso and convert to meters
	-- TODO: add rotation
	for j=1,15 do
		sk_pos[j] = (sk_pos[j]-sk_pos[NITE_JOINT_TORSO])/1000
	end
	
	-- Foot offset
	local offground_l = sk_pos[NITE_JOINT_LEFT_FOOT][3]+.75;
	local offground_r = sk_pos[NITE_JOINT_RIGHT_FOOT][3]+.75;
	local base_z = 0;
	local base_foot = 0; --left
	if offground_l>offground_r then
		base_foot = 1;
		base_z = offground_r
	else
		base_z = offground_l
	end

	-- Vector differences
	local s2eL = sk_pos[NITE_JOINT_LEFT_SHOULDER] - sk_pos[NITE_JOINT_LEFT_ELBOW]
	local e2hL = sk_pos[NITE_JOINT_LEFT_ELBOW] - sk_pos[NITE_JOINT_LEFT_HAND]
	local s2eR = sk_pos[NITE_JOINT_RIGHT_SHOULDER]-sk_pos[NITE_JOINT_RIGHT_ELBOW]
	local e2hR = sk_pos[NITE_JOINT_RIGHT_ELBOW] - sk_pos[NITE_JOINT_RIGHT_HAND]
	local s2s = sk_pos[NITE_JOINT_RIGHT_SHOULDER]-sk_pos[NITE_JOINT_LEFT_SHOULDER]
	local t2n = sk_pos[NITE_JOINT_NECK]-sk_pos[NITE_JOINT_TORSO]
	
	-- Body Yaw
	local body_yaw = math.atan2(s2s[1],s2s[2]);
	print('Yaw',body_yaw)
	
	-- Elbow
	local elbow_l = math.acos( s2eL*e2hL / (vector.norm(s2eL)*vector.norm(e2hL)) )
	local elbow_r = math.acos( s2eR*e2hR / (vector.norm(s2eR)*vector.norm(e2hR)) )
	
	-- Solve IK for shoulders
	-- Pitch
	local shoulder_pitch_l = math.atan2(s2eL[3],s2eL[1])
	local shoulder_pitch_r = math.atan2(s2eL[3],s2eL[1])
	
	-- Roll
	local shoulder_roll_l = math.atan2( s2eL[2], math.sqrt(s2eL[1]^2+s2eL[3]^2) )
	local shoulder_roll_r = math.atan2( s2eR[2], math.sqrt(s2eR[1]^2+s2eR[3]^2) )

	-- Yaw
	local zeroTrans1 = Transform.rotY(shoulder_pitch_l)
	s2eL[4] = 1;
	local pLArm1 = zeroTrans1 * s2eL;
	local zeroTrans2 = Transform.rotZ( -1 * shoulder_roll_l )
	local pLArm2 = zeroTrans2 * pLArm1;
	e2hL[4] = 1;
	local yawdL = zeroTrans2 * zeroTrans1 * e2hL;
	local yawL  = math.atan2( yawdL[2], -1 * yawdL[3]);
	--
	zeroTrans1 = Transform.rotY(shoulder_pitch_r)
	s2eR[4] = 1;
	local pRArm1 = zeroTrans1 * s2eR;
	zeroTrans2 = Transform.rotZ( -1 * shoulder_roll_r )
	local pRArm2 = zeroTrans2 * pRArm1;
	e2hR[4] = 1;
	local yawdR = zeroTrans2 * zeroTrans1 * e2hR;
	local yawR  = math.atan2( yawdR[2], -1 * yawdR[3]);
	
	-- Send to webots
	wbJoints[ Params.jointID["L_Elbow"] ]          = elbow_l
	wbJoints[ Params.jointID["L_Shoulder_Pitch"] ] = 
		-1*(shoulder_pitch_l-math.pi/2)
	wbJoints[ Params.jointID["L_Shoulder_Roll"] ]  = -1 * shoulder_roll_l
	wbJoints[ Params.jointID["L_Shoulder_Yaw"] ]   = -1*yawL
	--
	----[[
	wbJoints[ Params.jointID["R_Elbow"] ]          = -1 * elbow_r	

	wbJoints[ Params.jointID["R_Shoulder_Pitch"] ] = 
		(shoulder_pitch_r - math.pi/2)
		
	wbJoints[ Params.jointID["R_Shoulder_Roll"] ]  = -1 * shoulder_roll_r
	wbJoints[ Params.jointID["R_Shoulder_Yaw"] ]   = yawR
	--]]
	
	-- Inverse legs
	--local pTorso = vector.new({supportX, 0, bodyHeight, 0,bodyTilt,0});
	local height_l = (sk_pos[NITE_JOINT_LEFT_HIP] - sk_pos[NITE_JOINT_LEFT_FOOT])
	*.85/.92
	local height_r = (sk_pos[NITE_JOINT_RIGHT_HIP]-sk_pos[NITE_JOINT_RIGHT_FOOT])
	*.85/.92
	local h = (height_l+height_r)/2
	
	local df_l = sk_pos[NITE_JOINT_TORSO] - sk_pos[NITE_JOINT_LEFT_FOOT]
	local df_r = sk_pos[NITE_JOINT_TORSO] - sk_pos[NITE_JOINT_RIGHT_FOOT]
	
	local pTorso = vector.new( {0, 0,       h[3],  0,0,0} );
	local pLeg_l = vector.new( {0, -1*df_l[2], 0,     0,0,0} );
	local pLeg_r = vector.new( {0, -1*df_r[2], 0,     0,0,0} );
	
	if base_foot==0 and math.abs(offground_r)>.05 then
		--print("left foot",offground_r)
	elseif base_foot==1 and math.abs(offground_l)>.05 then
		print("right foot",offground_l)
		pLeg_l = vector.new({0, df_l[2], offground_l,   0,0,0});
	end
	
	local ik_legs = K.inverse_legs( pLeg_l, pLeg_r,	pTorso, base_foot );
	local jj = 1;
	local use_torso = true;
	if use_torso then
		for j=Params.jointID["L_Hip_Yaw"],Params.jointID["R_Ankle_Roll"] do
			wbJoints[j] = ik_legs[jj]
			jj = jj+1
		end
	end
end

run()

assert( openni.shutdown() )