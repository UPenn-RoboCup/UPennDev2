--Step controller based on reactive ZMP walking


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libReactiveZMP = require'libReactiveZMP'
local zmp_solver

local libStep = require'libStep'
local step_planner

require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep
local stepHeight  = Config.walk.stepHeight

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep

-- What foot trajectory are we using?
local foot_traj_func  
if Config.walk.foot_traj==1 then foot_traj_func = moveleg.foot_trajectory_base
else foot_traj_func = moveleg.foot_trajectory_square end

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

  -- Initiate the ZMP solver
  zmp_solver = libReactiveZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,    
  })

  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()

  tStep = Config.walk.tStep
  --Now we advance a step at next update  
  t_last_step = Body.get_time() - tStep 

  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag


--Test steps 

  --uFoot supportLeg tStep
  step_planner:step_enque({0.10,-0.10,0},0,0.5) --LS
  step_planner:step_enque({0.10, 0.10,0},1,1) --RS
  step_planner:step_enque({0.10,-0.10,0},0,2) --LS
  step_planner:step_enque({0.05, 0.10,0},1,0.5) --LS

end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time

  -- Grab the phase of the current step
  local ph = (t-t_last_step)/tStep
  if ph>1 then
    --Should we stop now?
    local stoprequest = mcm.get_walk_stoprequest()
    if stoprequest>0 then return"done"   end
    ph = ph % 1
    iStep = iStep + 1  -- Increment the step index  

    local initial_step = false
--  if iStep<=3 then initial_step = true end
    
    step_planner:save_stance(uLeft_next,uRight_next,uTorso_next)

    --Calculate next step and torso positions based on velocity      
    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport ,supportLeg, tStep=
      step_planner:get_next_step_queue(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step)

    if not uLeft_now then --All step done
      return "done"    
    end  

    zmp_solver:set_param(tStep)

    -- Compute the ZMP coefficients for the next step
    zmp_solver:compute( uSupport, uTorso_now, uTorso_next )
    t_last_step = Body.get_time() -- Update t_last_step
    print("new tStep:",tStep)
  end

  local uTorso = zmp_solver:get_com(ph)
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

  local phSingle = moveleg.get_ph_single(ph,Config.walk.phSingle[1],Config.walk.phSingle[2])
--  if iStep<=2 then phSingle = 0 end --This kills compensation and holds the foot on the ground  
  local uLeft, uRight, zLeft, zRight = uLeft_now, uRight_now, 0,0
  if supportLeg == 0 then  -- Left support    
    uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight)    
  else    -- Right support    
    uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight)    
  end

  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )
  delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,phSingle,gyro_rpy, angleShift)
  moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')  
end

return walk
