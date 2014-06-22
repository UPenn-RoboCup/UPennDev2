local t_entry, t_update
local state = {}
state._NAME = ...

local Body = require'Body'
local vector = require'vector'
local HT = require'libHeadTransform'
local util = require'util'
require'wcm'
require'gcm'

local ball_radius = Config.world.ballDiameter / 2
local tLost = Config.fsm.headTrack.tLost
local timeout = Config.fsm.headTrack.timeout
local dqNeckLimit = Config.fsm.dqNeckLimit

local pitchMin = Config.head.pitchMin
local pitchMax = Config.head.pitchMax
local yawMin = Config.head.yawMin
local yawMax = Config.head.yawMax

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

  if t-t_entry > timeout then
   
      return 'timeout' 

  end

  local ball_elapsed = t - wcm.get_ball_t()

  if ball_elapsed > tLost then --ball lost
    print "Ball lost"
    return 'balllost'
  end

  local ballX, ballY = wcm.get_ball_x(), wcm.get_ball_y()
  local yaw, pitch = HT.ikineCam(ballX, ballY, ball_radius)

  -- Clamp
  yaw = math.min(math.max(yaw, yawMin), yawMax)
  pitch = math.min(math.max(pitch, pitchMin), pitchMax)

  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, {yaw,pitch}, dqNeckLimit, dt )
    
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
