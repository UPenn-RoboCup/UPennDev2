local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'
require'wcm'
require'gcm'

local vector = require'vector'
local ball_radius = Config.world.ballDiameter / 2

local util = require'util'
local HeadTransform = require'HeadTransform'

local tLost = Config.fsm.headTrack.tLost
local timeout = Config.fsm.headTrack.timeout

-- Neck limits
--TODO: put into Config
local dqNeckLimit = vector.new{15, 15} * DEG_TO_RAD
if IS_WEBOTS then
  dqNeckLimit = vector.new{45,45} * DEG_TO_RAD
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
  local yaw, pitch = HeadTransform.ikineCam( ballX, ballY, ball_radius)

  --TODO: a hack
  -- when ball is close to body, look down to avoid losing the visual
  local ballR = math.sqrt(ballX*ballX + ballY*ballY)
  if ballR < 0.3 then pitch = pitch + 5*DEG_TO_RAD end

  -- Grab where we are
  local qNeck = Body.get_head_position()
  local qNeck_approach, doneNeck =
    util.approachTol( qNeck, {yaw, pitch}, dqNeckLimit, dt )

  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

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
