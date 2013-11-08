--Step controller based on preview ZMP walking


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libZMP = require'libPreviewZMP'
local zmp_solver

local libStep = require'libStep'
local step_planner

require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep
local stepHeight  = Config.walk.stepHeight

local zLeft,zRight --Step landing heights

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep

-- What foot trajectory are we using?
local foot_traj_func  
--foot_traj_func = moveleg.foot_trajectory_base
--foot_traj_func = moveleg.foot_trajectory_square
foot_traj_func = moveleg.foot_trajectory_square_stair
--foot_traj_func = moveleg.foot_trajectory_square_stair_2




local t, t_discrete

local debugdata
local t0

local read_test = false
--local read_test = true
local debug_on = false




---------------------------
-- State machine methods --
---------------------------
function walk.entry()
  print(walk._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
 
  mcm.set_walk_vel({0,0,0})--reset target speed

  tStep = Config.walk.tStep

  -- Initiate the ZMP solver
  zmp_solver = libZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,    
  })
  zmp_solver:precompute()

  step_planner = libStep.new_planner()

  zLeft, zRight = 0,0
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()

  zmp_solver:init_preview_queue(uLeft_now,uRight_now, uTorso_now, Body.get_time(), step_planner)
  
  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
  mcm.set_walk_ismoving(1) --We started moving
  --Reset odometry varialbe
  Body.init_odometry(uTorso_now)  

  --SHM BASED
  local nFootHolds = mcm.get_step_nfootholds()
  local footQueue = mcm.get_step_footholds()
  print("step #:",nFootHolds)

  for i=1,nFootHolds do
    local offset = (i-1)*13;
    local foot_movement = {footQueue[offset+1],footQueue[offset+2],footQueue[offset+3]}
    local supportLeg = footQueue[offset+4]
    local t0 = footQueue[offset+5]
    local t1 = footQueue[offset+6]
    local t2 = footQueue[offset+7]
    local zmp_mod = {footQueue[offset+8],footQueue[offset+9],footQueue[offset+10]}
    local footparam = {footQueue[offset+11],footQueue[offset+12],footQueue[offset+13]}    
    step_planner:step_enque_trapezoid(foot_movement, supportLeg, t0,t1,t2,zmp_mod,footparam)
  end

  t = Body.get_time()
  time_discrete_shift = zmp_solver:trim_preview_queue(step_planner,t )  
  t_discrete = t 
  t0 = t

  debugdata=''
 
  if read_test then
    Body.request_lleg_position()
    Body.request_rleg_position()
  end
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time
  local discrete_updated = false
  local com_pos

  while t_discrete<t do
    zmp_solver:update_preview_queue_steps(step_planner,t_discrete + time_discrete_shift)
    t_discrete = t_discrete + zmp_solver.preview_tStep
    discrete_updated = true

    --Get step information
    uLeft_now, uRight_now, uLeft_next, uRight_next,
      supportLeg, ph, ended, walkParam = zmp_solver:get_current_step_info(t_discrete + time_discrete_shift)

    if ended and zmp_solver:can_stop() then return "done"  end
  
    --Get the current COM position
    com_pos,zmp_pos = zmp_solver:update_state()
  
  end

  if discrete_updated then
    local uTorso = {com_pos[1],com_pos[2],0}
    uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

    --Calculate Leg poses 
    local phSingle = moveleg.get_ph_single(ph,Config.walk.phSingle[1],Config.walk.phSingle[2])
    local uLeft, uRight = uLeft_now, uRight_now
    
    if supportLeg == 0 then  -- Left support    
      uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight,walkParam)    
--      if walkParam then print(unpack(walkParam))end
    elseif supportLeg==1 then    -- Right support    
      uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight,walkParam)    
--      if walkParam then print(unpack(walkParam))end
    elseif supportLeg == 2 then --Double support
    end
    step_planner:save_stance(uLeft,uRight,uTorso)  

    --Update the odometry variable
    Body.update_odometry(uTorso)

    local uZMP = zmp_solver:get_zmp()
    mcm.set_status_uTorso(uTorso)
    mcm.set_status_uZMP(uZMP)
    mcm.set_status_t(t)


    --Calculate how close the ZMP is to each foot
    local uLeftSupport,uRightSupport = 
      step_planner.get_supports(uLeft,uRight)
    local dZmpL = math.sqrt(
      (uZMP[1]-uLeftSupport[1])^2+
      (uZMP[2]-uLeftSupport[2])^2);

    local dZmpR = math.sqrt(
      (uZMP[1]-uRightSupport[1])^2+
      (uZMP[2]-uRightSupport[2])^2);

    local supportRatio = dZmpL/(dZmpL+dZmpR);

--print(unpack(uTorso),unpack(uLeft),unpack(uRight))

  -- Grab gyro feedback for these joint angles
    local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )

    --delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,ph,gyro_rpy, angleShift)

    delta_legs, angleShift = moveleg.get_leg_compensation_new(
      supportLeg,
      ph,
      gyro_rpy, 
      angleShift,
      supportRatio)

    --Move legs
    local uTorsoComp = mcm.get_stance_uTorsoComp()
    local uTorsoCompensated = util.pose_global(
      {uTorsoComp[1],uTorsoComp[2],0},uTorso)

    moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  
      zLeft,zRight,delta_legs)    
  end

  if debug_on then

    local qLLeg, qRLeg
    if read_test then
      qLLeg = Body.get_lleg_position()
      qRLeg = Body.get_rleg_position()
      Body.request_lleg_position()
      Body.request_rleg_position()
    else
      qLLeg = Body.get_lleg_command_position()
      qRLeg = Body.get_rleg_command_position()
    end
    local qLLegCommand = Body.get_lleg_command_position()
    local qRLegCommand = Body.get_rleg_command_position()
    local rpy = Body.get_sensor_rpy()
    debugdata=debugdata..
    string.format("%f,  %f,%f,%f,%f,%f,  %f,%f,%f,%f,%f,  %f,%f,  %f,%f,%f,%f,%f,  %f,%f,%f,%f,%f\n",      
      t-t0,
      qRLeg[2]*Body.RAD_TO_DEG,
      qRLeg[3]*Body.RAD_TO_DEG,
      qRLeg[4]*Body.RAD_TO_DEG,
      qRLeg[5]*Body.RAD_TO_DEG,
      qRLeg[6]*Body.RAD_TO_DEG,

      qRLegCommand[2]*Body.RAD_TO_DEG,
      qRLegCommand[3]*Body.RAD_TO_DEG,
      qRLegCommand[4]*Body.RAD_TO_DEG,
      qRLegCommand[5]*Body.RAD_TO_DEG,
      qRLegCommand[6]*Body.RAD_TO_DEG,                 
      
      rpy[1]*Body.RAD_TO_DEG,
      rpy[2]*Body.RAD_TO_DEG,      

      qLLeg[2]*Body.RAD_TO_DEG,
      qLLeg[3]*Body.RAD_TO_DEG,
      qLLeg[4]*Body.RAD_TO_DEG,
      qLLeg[5]*Body.RAD_TO_DEG,
      qLLeg[6]*Body.RAD_TO_DEG,

      qLLegCommand[2]*Body.RAD_TO_DEG,
      qLLegCommand[3]*Body.RAD_TO_DEG,
      qLLegCommand[4]*Body.RAD_TO_DEG,
      qLLegCommand[5]*Body.RAD_TO_DEG,
      qLLegCommand[6]*Body.RAD_TO_DEG               
      )
      print("Roll: ",rpy[1]*Body.RAD_TO_DEG," Pitch:",rpy[2]*Body.RAD_TO_DEG)
  end
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')  
  mcm.set_walk_ismoving(0) --We stopped moving
  if debug_on then
    local savefile = string.format("Log/debugdata_%s",os.date());
    local debugfile=assert(io.open(savefile,"w")); 
    debugfile:write(debugdata);
    debugfile:flush();
    debugfile:close();  
  end
  print("DONE")
end

return walk
