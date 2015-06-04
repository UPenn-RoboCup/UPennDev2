local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
require'hcm'
require'wcm'
require'mcm'

local footstepplanner = require'footstepplanner'

-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')

local t_entry, t_update, t_exit, t_stage, step_queues
local nwaypoints, wp_id
local waypoints = {}

local pose, target_pose
local step_planner
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg




local stage = 1
local ready_for_input = true





local function calculate_footsteps(stage)
  local step_queue = step_queues[stage]
  --Write to SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(15*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*15;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]

    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    
    step_queue_vector[offset+7] = step_queue[i][5]    

    step_queue_vector[offset+8] = step_queue[i][6][1]
    step_queue_vector[offset+9] = step_queue[i][6][2]
    step_queue_vector[offset+10] = 0

    step_queue_vector[offset+11] = step_queue[i][7][1]
    step_queue_vector[offset+12] = step_queue[i][7][2]
    step_queue_vector[offset+13] = step_queue[i][7][3]

    if step_queue[i][8] then
      step_queue_vector[offset+14] = step_queue[i][8][1]
      step_queue_vector[offset+15] = step_queue[i][8][2]
    else
      step_queue_vector[offset+14] = 0
      step_queue_vector[offset+15] = 0
    end

  end
  mcm.set_step_footholds(step_queue_vector)
  mcm.set_step_nfootholds(#step_queue)
end


function initiate_step(supportLeg, step_relpos )
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  --Which foot is ahead?
  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso) 

  local step_min = 0.05
  local sh1,sh2 = 0.05, 0

  local st=1
  local wt = Config.slowstep_duration or 3
 --slower step




--F/W max: 0.20
--sidesteo max: 0.10
  local supportYSS = Config.supportYSS or -0.02

  local move_target = vector.pose(hcm.get_teleop_waypoint())



  local step_type
  if Config.birdwalk then
    if move_target[3]==0 then
      if move_target[2]>=0 then step_type="left2" 
      else step_type="right2" end
    elseif move_target[3]>0 then
      move_target[2]= math.min(0,move_target[2])  
      step_type="right2"
    else
      move_target[2]= math.max(0,move_target[2])  
      step_type="left2"
    end
  else
    if move_target[3]==0 then
      if move_target[2]>=0 then step_type="left2" 
      else step_type="right2" end
    elseif move_target[3]<0 then
      move_target[2]= math.min(0,move_target[2])  
      step_type="right2"
    else
      move_target[2]= math.max(0,move_target[2])  
      step_type="left2"
    end
  end


  --Sanitize the velocity
  local step_lim = {0.15,0.08,15*math.pi/180}

  local step_mag = math.sqrt((move_target[1]/step_lim[1])^2+(move_target[2]/step_lim[2])^2)
  step_mag = math.max(1,step_mag)

  move_target[1] = move_target[1]/step_mag
  move_target[2] = move_target[2]/step_mag
  move_target[3] = math.min(step_lim[3], math.max(-step_lim[3],move_target[3]))

  print("ALIGN STEP:",move_target[1],move_target[2],move_target[3]*RAD_TO_DEG)

  local uTorsoTarget = util.pose_global(move_target, uTorso)
  local uLeftTarget = util.pose_global({0,Config.walk.footY,0},uTorsoTarget)
  local uRightTarget = util.pose_global({0,-Config.walk.footY,0},uTorsoTarget)



  local uLeftSupport = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeft)
  local uRightSupport = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRight)
  

  --if we take the left step first 
  --this will be the first target torso position
  local uTorsoMidLeft = util.se2_interpolate(0.5,uLeftTarget,uRight)
  local uRightTorsoMidLeft = util.pose_relative(uRight, uTorsoMidLeft)
  local uLeftTargetTorsoMidLeft = util.pose_relative(uLeftTarget, uTorsoMidLeft)


  local uTorsoMidRight = util.se2_interpolate(0.5,uLeft,uRightTarget)
  local uLeftTorsoMidRight = util.pose_relative(uLeft, uTorsoMidRight)
  local uRightTargetTorsoMidRight = util.pose_relative(uRightTarget, uTorsoMidRight)


  local uLeftMove = util.pose_relative(uLeftTarget,uLeft)
  local uRightMove = util.pose_relative(uRightTarget,uRight)

  local side_adj = Config.walk.supportY - 0.00
  local com_side = Config.walk.footY+Config.walk.supportY-side_adj

  local lt = 0.1


print("step type:",step_type)
  if step_type=="left2" then
      --take left step
      local move1 = math.sqrt(uRightTorso[1]^2+uRightTorso[2]^2)
      local uTorsoMove = util.pose_relative(uTorsoTarget,uRight)
      local move2 = math.sqrt(uTorsoMove[1]^2+uTorsoMove[2]^2)

print("torso move dist:",move1,move2)

      step_queues={
         {
          {{0,0,0},2,        st, 0.1, 0.1,   {uRightTorso[1]  , uRightTorso[2] - supportYSS},{0,0,0} },    --Shift and Lift
          {uLeftMove,1,       0.1,wt,0.1 ,   {0,side_adj-supportYSS}, {0,sh1,sh2}   ,  {uRightTorsoMidLeft[1]  , uRightTorsoMidLeft[2] + Config.walk.supportY - supportYSS}},   --LS     --Move and land

          {{0,0,0}, 2,     st*2, 0.1, 0.1,   {uLeftTargetTorsoMidLeft[1],uLeftTargetTorsoMidLeft[2]+supportYSS},{0,0,0} },    --Shift and Lift

          {uRightMove,0,    lt,wt,lt ,     {0,-side_adj+supportYSS}, {0,sh1,sh2},  {0, Config.walk.footY-Config.walk.supportY+supportYSS}},   --LS     --Move and land
          {{0,0,0},2,        st, 0.1, 0.1,   {0,0},{0,0,0} },  --move to center
   
         },
       }
  elseif step_type=="right2" then
        --take right step
        step_queues={
         {
          {{0,0,0},    2,  st, 0.1, 0.1,   {uLeftTorso[1],uLeftTorso[2]},{0,0,0} },    --Shift and Lift
          {uRightMove,0,  0.1,wt,0.1 ,     {0,-side_adj},     {0,sh1,sh2},  {uLeftTorsoMidRight[1], uLeftTorsoMidRight[2] - Config.walk.supportY}},   --LS     --Move and land

          {{0,0,0}, 2,     st*2, 0.1, 0.1,   {uRightTargetTorsoMidRight[1],uRightTargetTorsoMidRight[2]},{0,0,0} },    --Shift and Lift

          {uLeftMove,1,   lt,wt,lt ,   {0,side_adj}, {0,sh1,sh2}   ,  {0, -Config.walk.footY+Config.walk.supportY}},   --LS     --Move and land
          {{0,0,0},2,        st, 0.1, 0.1,   {0,0},{0,0,0} },  --move to center      
         },
        }
  end
  stage = 1
  calculate_footsteps(stage)
  motion_ch:send'slowstep'  
  mcm.set_stance_singlesupport(1)
  is_started = true  
end







function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  t_stage = t_entry

  supportLeg=hcm.get_step_supportLeg()
  step_relpos = hcm.get_step_relpos() 
  initiate_step(supportLeg, step_relpos )
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if mcm.get_walk_ismoving()==0 and t-t_stage>0.5 then    
    if ready_for_input then print("Ready")
      ready_for_input = false
    end    
    if stage==#step_queues then 
      motion_ch:send'stop'  
      hcm.set_step_dir(0)
      return 'done'
    elseif hcm.get_state_proceed()==1 then     
      print("STEPADVANCE")
      --Clear the zmp compensation value here between transition---------------------
      local uTorsoZMPComp = mcm.get_status_uTorsoZMPComp()
      local uTorso = mcm.get_status_uTorso()
      uTorso = util.pose_global({uTorsoZMPComp[1],uTorsoZMPComp[2],0},uTorso)
      mcm.set_status_uTorsoZMPComp({0,0,0})
      mcm.set_status_uTorso(uTorso)
      hcm.set_state_proceed(0) --stop at each step
      stage = stage+1
      calculate_footsteps(stage)
      motion_ch:send'stepflat'  
      ready_for_input = true
      t_stage = t
    end
  else 
    hcm.set_state_proceed(0) --no pogo stick
  end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
