local HeadTransform = {}
local T = require'Transform'
local Body = require'Body'

local acos = require'math'.acos
local sqrt = require'math'.sqrt
local atan2 = require'math'.atan2

-- TODO: Change the reference frame to the torso, as the kinematics does this

-- Configuration
--[[
local dtrCamera = T.trans(unpack(Config.head.cameraPos or {0,0,0}))
  * T.rotY(Config.head.cameraPitch or 0)
  * T.rotX(Config.head.cameraRoll or 0)
local trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
	* T.rotY(Config.vision.bodyTilt)
	* T.trans(Config.head.neckX, 0, Config.head.neckZ)
--]]
-- Grab all the Config values
local cam_z = Config.head.cameraPos[3]
local cameraPitch = Config.head.cameraPitch
local trNeckOffset = T.trans(Config.head.neckX, 0, Config.head.neckZ)
local trCameraOffset = T.trans(unpack(Config.head.cameraPos))
-- TODO: Should not use the body transform in the HeadTransform, since should be with the torso as reference
local trBody = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)

-- TODO: No head bias yet
--[[
require'hcm'
hcm.set_camera_bias(Config.walk.headBias or {0,0,0})
--]]

-- Update the Head transform
local function get_head_position(head, rpy)  

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
  -- Grab the position only (4 coords, which includes scale)
  return T.position4(trHead)
end

--[[
v: vector of the position
--]]
local function projectGround(v, target_height)
	target_height = target_height or 0
  -- Grab head angles
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
  local vHead_homo = get_head_position(head, rpy)
  
  -- Project to plane
  if vHead[3] ~= target_height then
		local scale = (vHead[3] - target_height) / (vHead[3] - v[3])
    return vHead_homo + scale * (v - vHead_homo)
	else
		return vector.copy(v)
  end

end

function HeadTransform.project()
	return (trHead * v0) / v0[4]
end

function HeadTransform.ikineCam(x0, y0, z0)
  -- Grab head angles
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
	
  local vHead = get_head_position(head, rpy)
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
