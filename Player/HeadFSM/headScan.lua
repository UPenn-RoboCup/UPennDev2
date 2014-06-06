local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'
require'vcm'

local util = require'util'

-- Neck limits
local dqNeckLimit = Config.fsm.dqNeckLimit
local pitch0 = Config.fsm.headScan.pitch0
local pitchMag = Config.fsm.headScan.pitchMag
local yawMag = Config.fsm.headScan.yawMag

local tScan = Config.fsm.headScan.tScan
local timeout = tScan * 2

local t0
local direction = 1
local pitchDir = 1
local t_entry
function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t0 = Body.get_time()
  t_entry = t0
  t_update = t_entry
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if t - t_entry > timeout then
    return 'timeout'
  end

	-- Check if we found the ball
  local ball_elapsed = Body.get_time() - wcm.get_ball_t()
  if ball_elapsed < 0.1 then --ball found
    return 'ballfound'
  end

  local ph = (t - t0) / tScan
  ph = ph - math.floor(ph)

  local yaw, pitch
  if ph < 0.25 then --phase 0 to 0.25
    yaw = yawMag * (ph * 4) * direction
    pitch = pitch0 + pitchMag * pitchDir
  elseif ph < 0.75 then --phase 0.25 to 0.75
    yaw = yawMag * (1 - (ph - 0.25) * 4) * direction
    pitch = pitch0 - pitchMag * pitchDir
  else --phase 0.75 to 1
    yaw = yawMag * (-1 + (ph - 0.75) * 4) * direction
    pitch = pitch0 + pitchMag * pitchDir
  end

  local qNeck = Body.get_head_position()
  -- Go!
  local qNeck_approach, doneNeck =
    util.approachTol(qNeck, {yaw, pitch}, dqNeckLimit, dt)
--[[
print("YP", yaw, pitch)
print("act",qNeck_approach)
print("lim",unpack(dqNeckLimit))
--]]
  -- Update the motors
--  Body.set_head_command_position(qNeck_approach)
  Body.set_head_command_position({yaw, pitch})

end

function state.exit()
  print(state._NAME..' Exit')
end

return state

--[[

function entry()
  print("Head SM:".._NAME.." entry")
  wcm.set_ball_t_locked_on(0)

  --Goalie need wider scan
  role = gcm.get_team_role()
  if role==0 then
    yawMag=Config.fsm.headScan.yawMagGoalie
    mcm.set_walk_isSearching(0)
  else
    yawMag=Config.fsm.headScan.yawMag
  end

  -- start scan in ball's last known direction
  t0 = Body.get_time()
  ball = wcm.get_ball()
  timeout = tScan * 2

  yaw_0, pitch_0 = HeadTransform.ikineCam(ball.x, ball.y,0)
  local currentYaw = Body.get_head_position()[1]

  if currentYaw>0 then
    direction = 1
  else
    direction = -1
  end
  if pitch_0>pitch0 then
    pitchDir=1
  else
    pitchDir=-1
  end
  vcm.set_camera_command(-1) --switch camera
end

function update()
  pitchBias =  mcm.get_headPitchBias()--Robot specific head angle bias

  --Is the robot in bodySearch and spinning?
  isSearching = mcm.get_walk_isSearching()

  local t = Body.get_time()
  -- update head position

  -- Scan left-right and up-down with constant speed
  if isSearching ==0 then --Normal headScan

  else --Rotating scan
    timeout = 20.0 * Config.speedFactor --Longer timeout
    local ph = (t-t0)/tScan * 2
    ph = ph - math.floor(ph)
    --Look up and down in constant speed
    if ph<0.25 then
      pitch=pitchTurn0+pitchTurnMag*(ph*4)
    elseif ph<0.75 then
      pitch=pitchTurn0+pitchTurnMag*(1-(ph-0.25)*4)
    else
      pitch=pitchTurn0+pitchTurnMag*(-1+(ph-0.75)*4)
    end
    yaw = yawMagTurn * isSearching
  end

  Body.set_head_command({yaw, pitch-pitchBias})

  local ball = wcm.get_ball()
  if (t - ball.t < 0.1) then
    return "ball"
  end
  if (t - t0 > timeout) then
    return "timeout"
  end
end

function exit()
end
--]]
