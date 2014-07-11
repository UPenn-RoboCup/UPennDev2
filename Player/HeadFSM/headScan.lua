local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'


local t_entry, t_update
local stage
local DEG_TO_RAD = math.pi/180


--SJ: Adult sized league doesn't need too complicated stuff
--SO let's just define params directly on the fsm files

local dqNeckLimit = {90*DEG_TO_RAD,90*DEG_TO_RAD}

--Pitch: 25 degree down can see up to 5 meters 
--60 degree down can see ball right in front

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  stage = 1
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local qNeck0 = Body.get_head_command_position()
  local yawTarget, pitchTarget

  if stage==1 then
    pitchTarget = 55*DEG_TO_RAD
    yawTarget = 45*DEG_TO_RAD
  elseif stage==2 then
    pitchTarget = 55*DEG_TO_RAD
    yawTarget = -45*DEG_TO_RAD
  elseif stage==3 then
    pitchTarget = 25*DEG_TO_RAD        
    yawTarget = -45*DEG_TO_RAD
  elseif stage==4 then
    pitchTarget = 25*DEG_TO_RAD    
    yawTarget = 45*DEG_TO_RAD
  else
    print("role:",gcm.get_game_role())
    if gcm.get_game_role()==0 then
      --goalie shouldn't don't look behind
      stage = 1
    else
      --Attaker look around more
      return 'noball'
    end
  end

  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck0, {yawTarget,pitchTarget}, dqNeckLimit, dt )
  if doneNeck then stage = stage+1 end

  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

  -- Check if we found the ball
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed < 0.1 then
    return 'ballfound' --if ball found exit
  end
  
end

function state.exit()
  print(state._NAME..' Exit')
end

return state

