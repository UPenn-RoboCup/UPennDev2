local state = {}
state._NAME = ...

local Body = require'Body'
local HT = require'HeadTransform'
local util = require'util'
require'hcm'
require'vcm'
require'wcm'

local t0, t_update
local dqNeckLimit = Config.fsm.dqNeckLimit
local yawSweep = Config.fsm.headLookGoal.yawSweep;
local dist = Config.fsm.headReady.dist;
local tScan = Config.fsm.headLookGoal.tScan;
local minDist = Config.fsm.headLookGoal.minDist;
-- local min_eta_look = Config.min_eta_look or 2.0;
local yawMax = Config.head.yawMax or 90*Body.DEG_TO_RAD
local fovMargin = 30*Body.DEG_TO_RAD


function state.entry()
  print(state._NAME.." entry");
  t0 = Body.get_time();
  t_update = t0

  --SJ: Check which goal to look at
  --Now we look at the NEARER goal
  local pose = wcm.get_robot_pose()
  --TODO
  local defendGoal = {Config.world.goalLower[1][1],0}
  local attackGoal = {Config.world.goalUpper[1][1],0}

  local dDefendGoal= math.sqrt((pose[1]-defendGoal[1])^2 + (pose[2]-defendGoal[2])^2);
  local dAttackGoal= math.sqrt((pose[1]-attackGoal[1])^2 + (pose[2]-attackGoal[2])^2);
  local attackAngle = wcm.get_goal_attack_angle();
  local defendAngle = wcm.get_goal_defend_angle();

  --Can we see both goals?
  if math.abs(attackAngle)<yawMax + fovMargin and
     math.abs(defendAngle)<yawMax + fovMargin  then
    --Choose the closer one
    if dAttackGoal < dDefendGoal then
      yaw0 = attackAngle;
    else
      yaw0 = defendAngle;
    end
  elseif math.abs(attackAngle)<yawMax + fovMargin then
    yaw0 = attackAngle;
  elseif math.abs(defendAngle)<yawMax + fovMargin then
    yaw0 = defendAngle;
  else --We cannot see any goals from this position
    --We can still try to see the goals?
    if  math.abs(attackAngle) < math.abs(defendAngle) then
      yaw0 = attackAngle;
    else
      yaw0 = defendAngle;
    end
  end
end

function state.update()
  local t = Body.get_time();
  local tpassed=t-t0;
  local ph= tpassed/tScan;
  local yawbias = (ph-0.5)* yawSweep;

  local height = vcm.get_head_camera_height()

  local yaw1 = math.min(math.max(yaw0+yawbias, -yawMax), yawMax);
  local yaw, pitch = HT.ikineCam(
  	dist*math.cos(yaw1),dist*math.sin(yaw1), height);

  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local qNeck_approach, doneNeck =
    util.approachTol( qNeck, {yaw, pitch}, dqNeckLimit, tpassed )
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)


  if (t - t0 > tScan) then
    local tGoal = wcm.get_goal_t();
    if (tGoal - t0 > 0) then
      return 'timeout'
    else
      print('Goal lost!!')
      return 'lost'
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
