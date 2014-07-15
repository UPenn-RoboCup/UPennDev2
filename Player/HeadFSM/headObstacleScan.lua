local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'

local t_entry, t_update, stage
local dqNeckLimit = {20*DEG_TO_RAD,20*DEG_TO_RAD}
local tScan = Config.fsm.headObstacleScan.tScan
local yawMag = Config.fsm.headObstacleScan.yawMag

function state.entry()
	-- Enable obstacle detection
  print(state._NAME..' entry');
  t_entry = Body.get_time();
  t_update = t_entry
	stage = 0
	wcm.set_obstacle_enable(0)
	wcm.set_obstacle_reset(1)
end

function state.update()
  local t = Body.get_time()
  local dt = t-t_update
  t_update = t

  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck[1] = qNeck[1] - headBias[1]  

  --25 deg can basically cover the whole field
	-- but not enough for detecting obs near center circle
  local pitchUp = Config.fsm.headObstacleScan.pitchUp
  local pitchDown = Config.fsm.headObstacleScan.pitchDown
  local yaw

	if stage == 0 then 
    yaw = 0
    pitch = pitchUp
	elseif stage == 1 then 
    wcm.set_obstacle_enable(1)
		yaw = -yawMag
	elseif stage == 2 then 
    yaw = 0
    pitch = pitchDown
	elseif stage == 3 then 
		yaw = yawMag
	elseif stage == 4 then 
    yaw = 0
    pitch = pitchUp
	else 
  	wcm.set_obstacle_enable(0)
	return 'done' 
	end

  local qNeck_approach, doneNeck =
    util.approachTol(qNeck, {yaw, pitch}, dqNeckLimit, dt)

	if doneNeck then stage = stage+1 end

  if wcm.get_ball_t()>wcm.get_ball_tlook() then
    wcm.set_ball_tlook(wcm.get_ball_t())
  end

  -- Update the motors
--  Body.set_head_command_position(qNeck_approach)

  local headBias = hcm.get_camera_bias()
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
end

function state.exit()
	wcm.set_obstacle_enable(0)
  print('Obstacle detection disabled?', wcm.get_obstacle_enable())
end

return state
