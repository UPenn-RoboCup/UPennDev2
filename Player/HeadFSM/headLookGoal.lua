local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local HT = require'libHeadTransform'
require'hcm'
require'vcm'
require'wcm'

local t0, t_update, t_entry
local dqNeckLimit = Config.fsm.dqNeckLimit or {180*DEG_TO_RAD, 180*DEG_TO_RAD}
local yawSweep = Config.fsm.headLookGoal.yawSweep;
local dist = Config.fsm.headReady.dist;
local tScan = Config.fsm.headLookGoal.tScan;
local yawMax = Config.head.yawMax or 90*Body.DEG_TO_RAD
local fovMargin = 10*DEG_TO_RAD

local stage, scandir

function state.entry()
  print(state._NAME.." entry");
  t0 = Body.get_time();
  t_entry = Body.get_time()
  t_update = t0

  --SJ: Check which goal to look at
  --Now we look at the NEARER goal
  local pose = wcm.get_robot_pose()  
  
  local attackGoal = {Config.world.goalUpper[1][1],0}
  local defendGoal = {Config.world.goalLower[1][1],0}

  local dDefendGoal= math.sqrt((pose[1]-defendGoal[1])^2 + (pose[2]-defendGoal[2])^2);
  local dAttackGoal= math.sqrt((pose[1]-attackGoal[1])^2 + (pose[2]-attackGoal[2])^2);

  local attackAngle = util.mod_angle(
    math.atan2(attackGoal[2]-pose[2],attackGoal[1]-pose[1])-pose[3])
  local defendAngle = util.mod_angle(
    math.atan2(defendGoal[2]-pose[2],defendGoal[1]-pose[1])-pose[3])

--print("AttackAngle:",attackAngle)

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

  stage,scandir = 1,1
  local qNeck = Body.get_head_command_position()
  local yaw1 = yaw0-0.5*yawSweep
  local yaw2 = yaw0+0.5*yawSweep
  if math.abs(qNeck[1]-yaw1)>math.abs(qNeck[1]-yaw2) then scandir = -1 end
  t_update = Body.get_time()
end

function state.update()
  local t = Body.get_time();
  local tpassed=t-t_update
  t_update = t


  local qNeck = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck[1] = qNeck[1] - headBias[1]  

  local pitch = -5*DEG_TO_RAD
  if stage==1 then
    local yawbias = -0.5*yawSweep*scandir
    local yaw = math.min(math.max(yaw0+yawbias, -yawMax), yawMax)
    local qNeck_approach, doneNeck =util.approachTol( qNeck, {yaw, pitch}, dqNeckLimit, tpassed )
--    Body.set_head_command_position(qNeck_approach)    

    local headBias = hcm.get_camera_bias()
    Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
    if doneNeck then
      t0 = t
      stage = 2
    end
  else
    local tpassed1=t-t0
    local ph = math.min(1,tpassed1/tScan)    
    local yawbias = (ph-0.5) * yawSweep*scandir

    local yaw = math.min(math.max(yaw0+yawbias, -yawMax), yawMax)
    local qNeck_approach, doneNeck =util.approachTol( qNeck, {yaw, pitch}, dqNeckLimit, tpassed )
    --Body.set_head_command_position(qNeck_approach)
    local headBias = hcm.get_camera_bias()
    Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})

    if doneNeck and ph==1 then
      local tGoal = wcm.get_goal_t();
      if (tGoal - t0 > 0) then
        return 'timeout'
      else
        print('Goal lost!!')
        return 'lost'
      end
    end
  end
end

function state.exit()
  print(state._NAME..' Exit'..' total time:'..Body.get_time()-t_entry  )
end

return state
