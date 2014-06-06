local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'
require'wcm'
require'gcm'

local vector = require'vector'
local util = require'util'
local T = require'libTransform'

local ball_radius = Config.world.ballDiameter / 2
local tLost = Config.fsm.headTrack.tLost
local timeout = Config.fsm.headTrack.timeout
local dqNeckLimit = Config.fsm.dqNeckLimit

local pitchMin = Config.head.pitchMin
local pitchMax = Config.head.pitchMax
local yawMin = Config.head.yawMin
local yawMax = Config.head.yawMax

-- Assume one head for now...
local dtrCamera = T.trans(unpack(Config.head.cameraPos or {0,0,0}))
  * T.rotY(Config.head.pitchCamera or 0)
local trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
* T.rotY(Config.walk.bodyTilt)
* T.trans(Config.head.neckX, 0, Config.head.neckZ)

-- Update the Head transform
-- Input: Head angles
local function update_head()
  if not Body then return end
  -- Get from Body...
  local head = Body.get_head_position()
  -- TODO: Smarter memory allocation
  -- TODO: Add any bias for each robot
  trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
  trHead = trNeck * dtrCamera
  -- Grab the position only
  local vHead = T.get_pos(trHead)
  return vHead
end

--Camera IK without headangle limit
local function ikineCam0(x0, y0, z0)
  local pitch0 = 0
  --Look at ground by default
  z0 = z0 or 0

  local vNeck = update_head()
  x0 = x0 - vNeck[1]
  z0 = z0 - vNeck[3]

  -- Cancel out body tilt angle
  local v = T.rotY(-Config.walk.bodyTilt) * torch.Tensor{x0,y0,z0,1}
  v = v / v[4]

  local x, y, z = v[1], v[2], v[3]
  local yaw = math.atan2(y, x)
  local norm = math.sqrt(x^2 + y^2 + z^2)

  --new IKcam that takes camera offset into account
  -------------------------------------------------------------
  -- sin(pitch)x + cos (pitch) z = c , c=camera z offset
  -- pitch = atan2(x,z) - acos(b/r),  r= sqrt(x^2+z^2)
  -- r*sin(pitch) = z *cos(pitch) + c,
  -------------------------------------------------------------
  local c = Config.head.cameraPos[3]
  local r = math.sqrt(x ^ 2 + y ^ 2)
  local d = math.sqrt(r ^ 2 + z ^ 2)
  local p0 = math.atan2(r, z) - math.acos(c / (d + 1E-10))

  local pitch = p0 - Config.head.cameraAngle[2] - pitch0
  return yaw, pitch
end

local function ikineCam(x, y, z)
  yaw,pitch=ikineCam0(x,y,z)
  yaw = math.min(math.max(yaw, yawMin), yawMax)
  pitch = math.min(math.max(pitch, pitchMin), pitchMax)
  return yaw,pitch
end

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local ball_elapsed = Body.get_time()-wcm.get_ball_t()
  if ball_elapsed> tLost then --ball lost
    return 'balllost'
  end

  local ballX, ballY = wcm.get_ball_x(), wcm.get_ball_y()
  local yaw, pitch = ikineCam( ballX, ballY, ball_radius)

  --TODO: a hack
  -- when ball is close to body, look down to avoid losing the visual
  local ballR = math.sqrt(ballX*ballX + ballY*ballY)
  if ballR < 0.3 then pitch = pitch + 5*DEG_TO_RAD end

  -- Grab where we are
  local qNeck = Body.get_head_position()
  --[[
  local qNeck_approach, doneNeck =
    util.approachTol( qNeck, {yaw, pitch}, dqNeckLimit, dt )
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)
  --]]
  Body.set_head_command_position({yaw, pitch})

  if t-t_entry > timeout then
    if gcm.get_game_role() == 0 then -- Goalie
      -- return 'sweep'
    else
      -- return 'timeout'
    end
  end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
