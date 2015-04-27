local HeadTransform = {}
local T = require'Transform'
local Body = require'Body'

local acos = require'math'.acos
local sqrt = require'math'.sqrt
local atan2 = require'math'.atan2

-- Configuration
--[[
local dtrCamera = T.trans(unpack(Config.head.cameraPos or {0,0,0}))
  * T.rotY(Config.head.cameraPitch or 0)
  * T.rotX(Config.head.cameraRoll or 0)
local trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
	* T.rotY(Config.vision.bodyTilt)
	* T.trans(Config.head.neckX, 0, Config.head.neckZ)
--]]

-- Choose the camera to use
local USE_KINECT = true
local cam_z
local cameraPitch
local trCameraOffset
if USE_KINECT then
	cam_z = Config.kinect.mountOffset[2][3]
	cameraPitch = Config.kinect.mountOffset[1][2]
	trCameraOffset = T.trans(unpack(Config.kinect.mountOffset[2]))
else
	cam_z = Config.head.cameraPos[3]
	cameraPitch = Config.head.cameraPitch
	trCameraOffset = T.trans(unpack(Config.head.cameraPos))
end

-- Neck offset is always the same
local trNeckOffset = T.trans(unpack(Config.head.neckOffset))

-- TODO: No head bias yet
--[[
require'hcm'
hcm.set_camera_bias(Config.walk.headBias or {0,0,0})
--]]

-- Update the Head transform
local function get_head_transform(head, rpy)  

	local bH = mcm.get_stance_bodyHeight()
	local uTorso = mcm.get_stance_uTorsoComp()
	local trBody = T.trans(-uTorso[1]-Config.walk.footX, -uTorso[2], bH)
	--local trBody = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)

  local trNeck0 = trBody * T.rotY(rpy[2]) * trNeckOffset
	
	-- TODO: No head bias for now... seems out of order
	-- TODO: Add one routine to actually set the bias, rather then checking it each loop
	local headBias = vector.zeros(4)
	--[[
	local headBias = hcm.get_camera_bias()
	local cameraPitch = headBias[2]
	local cameraRoll = headBias[3]
	local cameraYaw = headBias[4]
	--]]
	-- Apply the biases
	head[1] = head[1] - headBias[1]
	
	local dtrCamera = trCameraOffset
  * T.rotY(headBias[2])
  * T.rotX(headBias[3])
  * T.rotZ(headBias[4])
	
	local trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
  
  local trHead = trNeck * dtrCamera
  return trHead
end

--[[
v: vector of the position
--]]
local function projectGround(v, target_height)
	target_height = target_height or 0
  -- Grab head angles
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
  local trHead = get_head_transform(head, rpy)
	local vHead_homo = T.position4(trHead)
  
  -- Project to plane
  if vHead[3] ~= target_height then
		local scale = (vHead[3] - target_height) / (vHead[3] - v[3])
    return vHead_homo + scale * (v - vHead_homo)
	else
		return vector.copy(v)
  end

end

function HeadTransform.project(v0)
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
	local trHead = get_head_transform(head, rpy)
	return (trHead * v0) / v0[4]
end

function HeadTransform.ikineCam(x0, y0, z0)
  -- Grab head angles
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
	
  local trHead = get_head_transform(head, rpy)
	local vHead = T.position(trHead)
  x0 = x0 - vHead[1]
  -- Assume looking forward (for goal detection)
  z0 = (z0 or vHead[3]) - vHead[3]

  -- Cancel out body tilt angle
	-- TODO: Should not need this rpy, since the frame should be the torso. Maybe just the waist pitch...
  --local v = T.rotY(-Config.walk.bodyTilt) * {x0, y0, z0, 1}
	local v = T.rotY(-rpy[2]) * {x0, y0, z0, 1}
  v = v / v[4]

  local x, y, z = v[1], v[2], v[3]
  local yaw = atan2(y, x)

  local r = sqrt(x ^ 2 + y ^ 2)
  local d = sqrt(r ^ 2 + z ^ 2)
  local p0 = atan2(r, z) - acos(cam_z/(d + 1e-10))

  local pitch = p0 - cameraPitch

  return yaw, pitch
end

return HeadTransform
