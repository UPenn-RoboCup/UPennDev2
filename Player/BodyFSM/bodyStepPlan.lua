local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local libStep = require'libStep'
local step_planner

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM',true)

local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}



local pose, target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg

local function calculate_footsteps()
  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()
  supportLeg = 0

  local step_queue={}

  local tSlope1 = Config.walk.tStep*Config.walk.phSingle[1]
  local tSlope2 = Config.walk.tStep*(1-Config.walk.phSingle[2])
  
  step_queue[1] = {{0,0,0},2, 1,1,{0,0,0},{0,0,0}}
  local step_queue_count = 1;


  for i=1,10 do
    step_planner.velCurrent = vector.new({0.05,0,0})

    local new_step
    supportLeg = 1-supportLeg
    step_queue_count = step_queue_count + 1
    initial_step = false
    last_step = false
    if i==10 then last_step = true end

    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_velocity(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step,last_step)
    local leg_movemet
    if supportLeg==0 then --Left support
      leg_movement = util.pose_relative(uRight_next,uRight_now)  
    else
      leg_movement = util.pose_relative(uLeft_next,uLeft_now)  
    end
    if i==1 then
      new_step={leg_movement, supportLeg, tSlope1, Config.walk.tStep-tSlope1-tSlope2,
           {0,0,0},{0,Config.walk.stepHeight,0}}
    else
      new_step={leg_movement, supportLeg, tSlope1+tSlope2, Config.walk.tStep-tSlope1-tSlope2,
            {0,0,0},{0,Config.walk.stepHeight,0}}
    end
    step_queue[step_queue_count]=new_step
  end

  step_queue[step_queue_count+1] = {{0,0,0},2, tSlope2,2,{0,0,0},{0,0,0}}

  --Write to the SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(12*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*12;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]
    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    

    step_queue_vector[offset+7] = step_queue[i][5][1]
    step_queue_vector[offset+8] = step_queue[i][5][2]
    step_queue_vector[offset+9] = step_queue[i][5][3]

    step_queue_vector[offset+10] = step_queue[i][6][1]
    step_queue_vector[offset+11] = step_queue[i][6][2]
    step_queue_vector[offset+12] = step_queue[i][6][3]
  end
  hcm.set_motion_footholds(step_queue_vector)
  hcm.set_motion_nfootholds(#step_queue)
end




function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the pose
  local pose = {0,0,0}
  pose = wcm.get_robot_pose();


--[[

  -- Grab the waypoints
  nwaypoints = hcm.get_motion_nwaypoints()
  print('# of waypoints:', nwaypoints)
  print('waypoints', unpack(hcm.get_motion_waypoints()))
  local raw_waypoints = vector.slice(hcm.get_motion_waypoints(),1,3*nwaypoints)

  -- Check the frame of reference
  local waypoint_frame = hcm.get_motion_waypoint_frame()

  local idx = 1
  for w=1,nwaypoints do
    local waypoint = vector.slice(raw_waypoints,idx,idx+2)
    if waypoint_frame==0 then
      -- If a relative frame, then convert to the global frame
      waypoint = util.pose_global(waypoint, pose)
    end
    -- Add to the waypoints table
    waypoints[w] = waypoint
    -- Increment the index
    idx = idx + 3
  end

  -- Start with the first waypoint
  wp_id = 1
--]]  
  calculate_footsteps()
  motion_ch:send'preview'  
end





function state.update()
  --print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if mcm.get_walk_ismoving()==0 then
    return 'done'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
