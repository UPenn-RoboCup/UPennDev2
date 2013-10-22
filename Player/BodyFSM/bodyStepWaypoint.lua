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

local target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg



local function robocup_follow( pose, target_pose)
  local maxStep = 0.05
  local maxTurn = 0.15
  local dist_threshold = 0.025
  local angle_threshold = .1

  -- Distance to the waypoint
  rel_pose = util.pose_relative(target_pose,pose)
  rel_pose[3] = util.mod_angle(rel_pose[3])

  local rel_dist = math.sqrt(rel_pose[1]*rel_pose[1] + rel_pose[2]*rel_pose[2])
 
  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_pose[2],rel_pose[1]))

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
  vStep[1] = .25 * rel_pose[1]
  vStep[2] = .25 * rel_pose[2]
  
  -- Reduce speed based on how far away from the waypoint we are
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep

--[[
  if rel_dist>0.5 then
    vStep[3] = util.procFunc(0.25*aTurn,0,.15)
  end
--]]
  vStep[3] = math.min(0.15, math.max(-0.15, .25 * rel_pose[3]))


  -- If we are close to the waypoint and have the right angle threshold, we are fine
  -- TODO: Only with the last point do we care about the angle
  --print('Relative distances',rel_dist,rel_wp.a*180/math.pi)
  if rel_dist<dist_threshold then
    if math.abs(rel_pose[3])<angle_threshold then
    -- if not the last waypoint, then we are done with this waypoint
      return {0,0,0}, true 
    else
      vStep[3] = .05*util.sign(rel_pose[3])
    end
  end
  return vStep, false
end
local function calculate_footsteps()
  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()
  supportLeg = 0

  local pose_initial = {uTorso_now[1],uTorso_now[2],uTorso_now[3]}

  local step_queue={}

  local tSlope1 = Config.walk.tStep*Config.walk.phSingle[1]
  local tSlope2 = Config.walk.tStep*(1-Config.walk.phSingle[2])
  
  step_queue[1] = {{0,0,0},2, 0.1,1,0.1,{0,0,0},{0,0,0}}
  local step_queue_count = 1;
  local num_steps = 6
  local arrived = false;

  local max_step_count = 30
  
  while step_queue_count<max_step_count and not arrived do
    if not arrived then
      if step_queue_count==1 then
        step_planner.velCurrent = vector.new({0.0,0,0})
      else
        step_planner.velCurrent,arrived = robocup_follow(uTorso_now,target_pose)
      end
    else
      step_planner.velCurrent = vector.new({0.0,0,0})
      last_step=true 
    end
    
    supportLeg = 1-supportLeg
    step_queue_count = step_queue_count + 1
    initial_step, last_step = false, false
    if step_queue_count==max_step_count then last_step = true end
        
    local new_step
    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_velocity(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step,last_step)
    local leg_movemet
    if supportLeg==0 then --Left support
      leg_movement = util.pose_relative(uRight_next,uRight_now)  
    else
      leg_movement = util.pose_relative(uLeft_next,uLeft_now)  
    end
    new_step={leg_movement, 
              supportLeg, 
              tSlope1, 
              Config.walk.tStep-tSlope1-tSlope2,
              tSlope2,
              {0,0,0},
              {0,Config.walk.stepHeight,0}}
    
    step_queue[step_queue_count]=new_step
  end

  local pose_end = {uTorso_next[1],uTorso_next[2],uTorso_next[3]}

  print("Target relative pose:",unpack(target_pose))
  print("Actual relative pose:",unpack(
    util.pose_relative(pose_end,pose_initial)
    ))



  step_queue[step_queue_count+1] = {{0,0,0},2,  0.1,1,0.1,{0,0,0},{0,0,0}}

  --Write to the SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(13*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*13;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]

    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    
    step_queue_vector[offset+7] = step_queue[i][5]    

    step_queue_vector[offset+8] = step_queue[i][6][1]
    step_queue_vector[offset+9] = step_queue[i][6][2]
    step_queue_vector[offset+10] = step_queue[i][6][3]

    step_queue_vector[offset+11] = step_queue[i][7][1]
    step_queue_vector[offset+12] = step_queue[i][7][2]
    step_queue_vector[offset+13] = step_queue[i][7][3]
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

  target_pose = waypoints[1] --Single-waypoint approach for now

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
























