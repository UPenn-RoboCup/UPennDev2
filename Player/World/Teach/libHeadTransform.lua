local HeadTransform = {}
local T = require'Transform'
local Body = require'Body'

-- Configuration
--[[
local dtrCamera = T.trans(unpack(Config.head.cameraPos or {0,0,0}))
  * T.rotY(Config.head.cameraPitch or 0)
  * T.rotX(Config.head.cameraRoll or 0)
local trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
	* T.rotY(Config.vision.bodyTilt)
	* T.trans(Config.head.neckX, 0, Config.head.neckZ)
--]]
local cam_z = Config.head.cameraPos[3]

-- TODO: No head bias
--[[
require'hcm'
hcm.set_camera_bias(Config.walk.headBias or {0,0,0})
--]]

-- Update the Head transform
local function update_head()  
  -- Grab head angles
  local head = Body.get_head_position() 
  local rpy = Body.get_rpy()
  local trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(rpy[2])
  * T.trans(Config.head.neckX, 0, Config.head.neckZ)
	
	-- TODO: No head bias for now... seems out of order
	local headBias = vector.zeros(4)
	local headBias = hcm.get_camera_bias()
	--[[
	local cameraPitch = headBias[2]
	local cameraRoll = headBias[3]
	local cameraYaw = headBias[4]
	--]]
	-- Apply the biases
	head[1] = head[1] - headBias[1]
	
	local dtrCamera = T.trans(unpack(Config.head.cameraPos))
  * T.rotY(headBias[2])
  * T.rotX(headBias[3])
  * T.rotZ(headBias[4])
	
	local trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
  
  local trHead = trNeck * dtrCamera
  -- Grab the position only
  local vHead = T.position(trHead)
  return vHead
end

function HeadTransform.ikineCam(x0, y0, z0)

  local vHead = update_head()
  x0 = x0 - vHead[1]
  -- Assume looking forward (for goal detection)
  z0 = (z0 or vHead[3]) - vHead[3]

  -- Cancel out body tilt angle
  --local v = T.rotY(-Config.walk.bodyTilt) * {x0, y0, z0, 1}
	local v = T.rotY(-Body.get_rpy()[2]) * {x0, y0, z0, 1}
  v = v / v[4]

  local x, y, z = v[1], v[2], v[3]
  local yaw = math.atan2(y, x)

  local r = math.sqrt(x ^ 2 + y ^ 2)
  local d = math.sqrt(r ^ 2 + z ^ 2)
  local p0 = math.atan2(r, z) - math.acos(cam_z/(d + 1e-10))

  local pitch = p0 - Config.head.cameraPitch

  return yaw, pitch
end

return HeadTransform
