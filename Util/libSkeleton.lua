local openni = require'openni'
local util = require'util'
local Transform = require 'Transform'

local libSkeleton = {}

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
local joint_table = {}
local last_seen = {}
local nUsers = 2

libSkeleton.entry = function(logfile)
  openni.enable_skeleton()
  nUsers = openni.startup(logfile)
  assert(nUsers==2,'Should be tracking 2 users maximum')
  -- Initialize the joint_table and other user information
  for u=1,nUsers do
    joint_table[u] = {}
    table.insert(last_seen,0)
  end
  
  -- Verify stream
  local depth_info, color_info = openni.stream_info()
  assert(depth_info.width==320,'Bad depth resolution')
  assert(color_info.width==320,'Bad color resolution')
end

libSkeleton.exit = function()
  -- Shutdown the skeleton tracker
  openni.shutdown()
end

libSkeleton.update = function()
  local visible = openni.update_skeleton()
  
  -- (Re-)Populate the joint table for each visible user
  for u,v in ipairs(visible) do
    if v then
      -- Update the last seen time
      last_seen[u] = unix.time()
      -- Update the joint table
      local jTable = joint_table[u]
      for j=1,nJoints do jTable[j] = openni.joint(u,j) end
    end
  end
  
  -- Return the visible users from that frame
  return visible
  
end

-- Return the joint table for a particular user
function libSkeleton.get_joint_table( user )
  return joint_table[user]
end

-- Processing functions
function libSkeleton.get_darwinop_arm_angles(user)
  local jTable = joint_table[user]
  -- Left Hand
  if #jTable==0 then return end

  local e2hL = vector.sub(
    jTable[NITE_JOINT_LEFT_ELBOW].position,
    jTable[NITE_JOINT_LEFT_HAND].position)
  -- Correct the coordinates from OpenNI to our body frame
  e2hL = vector.new({e2hL[3],-1*e2hL[1],e2hL[2]}) 
  
  local s2eL = vector.sub(
    jTable[NITE_JOINT_LEFT_SHOULDER].position,
    jTable[NITE_JOINT_LEFT_ELBOW].position)
  -- Correct the coordinates from OpenNI to our body frame
  s2eL = vector.new({s2eL[3],-1*s2eL[1],s2eL[2]})
  
  local e2hR = vector.sub(
    jTable[NITE_JOINT_RIGHT_ELBOW].position,
    jTable[NITE_JOINT_RIGHT_HAND].position)
  -- Correct the coordinates from OpenNI to our body frame
  e2hR = vector.new({e2hR[3],-1*e2hR[1],e2hR[2]}) 
  
  local s2eR = vector.sub(
    jTable[NITE_JOINT_RIGHT_SHOULDER].position,
    jTable[NITE_JOINT_RIGHT_ELBOW].position)
  -- Correct the coordinates from OpenNI to our body frame
  s2eR = vector.new({s2eR[3],-1*s2eR[1],s2eR[2]})

  -- Elbow calculations
  -- Find the angle of the elbows
  local l_ratio = s2eL*e2hL / (vector.norm(s2eL)*vector.norm(e2hL))
  local elL = math.acos( util.procFunc(l_ratio,0,1) )
  local r_ratio = s2eR*e2hR / (vector.norm(s2eR)*vector.norm(e2hR) )
  local elR = math.acos( util.procFunc(r_ratio,0,1) )

  -- Shoulder calculations
  -- XZ plane defines the shoulder pitch
  local spL = math.atan2(s2eL[3],s2eL[1]) or 0
  local spR = math.atan2(s2eR[3],s2eR[1]) or 0
  -- YZ plane defines the shoulder roll
  local srL = math.atan2( s2eL[2], math.sqrt(s2eL[1]^2+s2eL[3]^2) ) or 0
  local srR = math.atan2( s2eR[2], math.sqrt(s2eR[1]^2+s2eR[3]^2) ) or 0

  -- Organize the vector
  local qLArm = vector.new({spL,-1*srL,-1*elL})
  local qRArm = vector.new({spR,-1*srR,-1*elR})

  return qLArm, qRArm

end

function libSkeleton.get_thorop_arm_angles(user)
  local jTable = joint_table[user]
  -- Left Hand
  if #jTable==0 then return end

  local e2hL = vector.sub(
    jTable[NITE_JOINT_LEFT_ELBOW].position,
    jTable[NITE_JOINT_LEFT_HAND].position)
  -- Correct the coordinates from OpenNI to our body frame
  e2hL = vector.new({e2hL[3],-1*e2hL[1],e2hL[2]}) 
  
  local s2eL = vector.sub(
    jTable[NITE_JOINT_LEFT_SHOULDER].position,
    jTable[NITE_JOINT_LEFT_ELBOW].position)
  -- Correct the coordinates from OpenNI to our body frame
  s2eL = vector.new({s2eL[3],-1*s2eL[1],s2eL[2]})
  
  local e2hR = vector.sub(
    jTable[NITE_JOINT_RIGHT_ELBOW].position,
    jTable[NITE_JOINT_RIGHT_HAND].position)
  -- Correct the coordinates from OpenNI to our body frame
  e2hR = vector.new({e2hR[3],-1*e2hR[1],e2hR[2]}) 
  
  local s2eR = vector.sub(
    jTable[NITE_JOINT_RIGHT_SHOULDER].position,
    jTable[NITE_JOINT_RIGHT_ELBOW].position)
  -- Correct the coordinates from OpenNI to our body frame
  s2eR = vector.new({s2eR[3],-1*s2eR[1],s2eR[2]})
  
  -- Shoulder to shoulder
	local s2s = vector.sub(
    jTable[NITE_JOINT_RIGHT_SHOULDER].position,
    jTable[NITE_JOINT_LEFT_SHOULDER].position)
  
  -- Torso to neck
	local t2n = vector.sub(
    jTable[NITE_JOINT_NECK].position,
    jTable[NITE_JOINT_TORSO].position)
    
	-- Body Yaw
	local body_yaw = math.atan2(s2s[1],s2s[2])
	print('Yaw',body_yaw)

  -- Elbow calculations
  -- Find the angle of the elbows
  local l_ratio = s2eL*e2hL / (vector.norm(s2eL)*vector.norm(e2hL))
  local elL = math.acos( util.procFunc(l_ratio,0,1) )
  local r_ratio = s2eR*e2hR / (vector.norm(s2eR)*vector.norm(e2hR) )
  local elR = math.acos( util.procFunc(r_ratio,0,1) )

	-- Shoulder Pitch
	local spL = math.atan2(s2eL[3],s2eL[1])
	local spR = math.atan2(s2eL[3],s2eL[1])
	
	-- Shoulder Roll
	local srL = math.atan2( s2eL[2], math.sqrt(s2eL[1]^2+s2eL[3]^2) )
	local srR = math.atan2( s2eR[2], math.sqrt(s2eR[1]^2+s2eR[3]^2) )

	-- Shoulder Yaw
	local zeroTrans1 = Transform.rotY(spL)
	s2eL[4] = 1
	local pLArm1 = zeroTrans1 * s2eL
	local zeroTrans2 = Transform.rotZ( -1 * srL )
	local pLArm2 = zeroTrans2 * pLArm1
	e2hL[4] = 1
	local yawdL = zeroTrans2 * zeroTrans1 * e2hL
	local syL  = math.atan2( yawdL[2], -1 * yawdL[3])
	--
	zeroTrans1 = Transform.rotY(spR)
	s2eR[4] = 1
	local pRArm1 = zeroTrans1 * s2eR
	zeroTrans2 = Transform.rotZ( -1 * srR )
	local pRArm2 = zeroTrans2 * pRArm1
	e2hR[4] = 1
	local yawdR = zeroTrans2 * zeroTrans1 * e2hR
	local syR  = math.atan2( yawdR[2], -1 * yawdR[3])

  -- Organize the vector
  local qLArm = vector.new({spL,-1*srL,syL,-1*elL,0,0})
  local qRArm = vector.new({spR,-1*srR,syR,-1*elR,0,0})

  return qLArm, qRArm

end

function libSkeleton.get_torso_rotation()

  -- Shoulder to shoulder
	local s2s = vector.sub(
    jTable[NITE_JOINT_RIGHT_SHOULDER].position,
    jTable[NITE_JOINT_LEFT_SHOULDER].position)
  
  -- Torso to neck
	local t2n = vector.sub(
    jTable[NITE_JOINT_NECK].position,
    jTable[NITE_JOINT_TORSO].position)

  -- These should form an orthonormal basis, so normalize
  t2n = t2n / vector.norm(t2n)
  s2s = s2s / vector.norm(s2s)

  -- Find the cross product
  local chest = vector.cross(s2s,t2n)

  -- Remap the coordinates
  local M = torch.Tensor({
    {chest[3],chest[1],chest[2]},
    {s2s[3],s2s[1],s2s[2]},
    {t2n[3],t2n[1],t2n[2]}
    })

  -- Find the closest Orthonormal Matrix
  local Rtorso = M * ( (M:t()*M)^-1/2 or 1 )

  -- Grab Euler Angles for SJ 
--http://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation#Roll-Pitch-Yaw_Angles
  local r = math.atan2(Rtorso[3][2],Rtorso[3][3])
  local y = math.atan2(Rtorso[2][1],Rtorso[1][1])
  local p = math.atan2(-1*Rtorso[3][1],math.cos(y)*Rtorso[1][1]+math.sin(y)*Rtorso[2][1])

  -- Clamp them
  r = util.procFunc(r,0,10*math.pi/180)
  p = util.procFunc(p,0,20*math.pi/180)
  y = util.procFunc(y,0,30*math.pi/180)

  return vector.new({r,p,y})
end

return libSkeleton