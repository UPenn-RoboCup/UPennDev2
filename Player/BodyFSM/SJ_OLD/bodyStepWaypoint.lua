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

require'mcm'



-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')

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

  -- If we are close to the waypoint and have the right angle threshold, we are fine
  -- TODO: Only with the last point do we care about the angle
  --print('Relative distances',rel_dist,rel_wp.a*180/math.pi)
  if rel_dist<dist_threshold then
    if math.abs(rel_pose[3])<angle_threshold then
    -- if not the last waypoint, then we are done with this waypoint
      return {0,0,0}, true 
    else
    	vStep[3] = math.min(
         Config.walk.maxTurnSpeed, 
         math.max(-Config.walk.maxTurnSpeed,
         Config.walk.aTurnSpeed * rel_pose[3]))
    end
  end
  return vStep, false
end

local function calculate_footsteps()
  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()

  print("CUrrent uLeft:",unpack(uLeft_now))
  print("CUrrent uTorso:",unpack(uTorso_now))
  print("CUrrent uRight:",unpack(uRight_now))

  --Select initial foot based on velocity
  if target_pose[2]>0 then --sidestep to the left
    supportLeg = 1  -- Right support
  else --sidestep to the right
    supportLeg = 0  -- Left support
  end


  local pose_initial = {uTorso_now[1],uTorso_now[2],uTorso_now[3]}

  local step_queue={}

  local tSlope1 = Config.walk.tStep*Config.walk.phSingle[1]
  local tSlope2 = Config.walk.tStep*(1-Config.walk.phSingle[2])
  
  step_queue[1] = {{0,0,0},2, 0.1,1,0.1,{0,0,0},{0,0,0}}
  local step_queue_count = 1

  local arrived = false

  --TODO: only sidestep requires frequent pauseing

--  local max_step_count = 30
  local max_step_count = Config.walk.maxStepCount or 6

  
  while step_queue_count<max_step_count and not arrived do
    if step_queue_count<=2 then
      step_planner.velCurrent = vector.new({0.0,0,0})
    else
      step_planner.velCurrent,arrived = robocup_follow(uTorso_now,target_pose)
    end
        
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
		
		supportLeg = 1-supportLeg
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
  mcm.set_step_footholds(step_queue_vector)
  mcm.set_step_nfootholds(#step_queue)
end




local follow_stage

local function forward_follow(uLeft,uTorso,uRight,uTarget)
  local uTorsoMinusSupport = util.pose_global(
    vector.new({-Config.walk.supportX,0,0}),
      uTorso)

  local rel_pose = util.pose_relative(uTarget,uTorsoMinusSupport) --global frame
  local rel_foot = util.pose_relative(uLeft,uRight)

print("rel pose x:",rel_pose[1])


  local max_step_x = 0.04

  local vStep={0,0,0}  
  local arrived = false

  if math.abs(rel_foot[1])<0.001 then --Foot together
    if math.abs(rel_pose[1])<max_step_x then 
      vStep[1] = rel_pose[1]/2
    else
      vStep[1] = math.max(-max_step_x/2,math.min(max_step_x/2, rel_pose[1]))      
    end
    follow_stage = 1
  else
    if follow_stage==1 then
      local dist_to_go = rel_pose[1] - math.abs(rel_foot[1])/2
      vStep[1] = math.max(-max_step_x,math.min(max_step_x, dist_to_go))
      if math.abs(vStep[1])<max_step_x then 
        follow_stage=2
      end
    else
      vStep[1] = math.abs(rel_foot[1]) --foot together
      arrived = true
    end
  end
 
  return vStep,arrived
end



local function calculate_footsteps_new(self)
  follow_stage = 0
  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()


  local uTorso0 = util.pose_global({-Config.walk.supportX,0,0},uTorso_now)
  local target_pose_local = util.pose_relative(target_pose,uTorso0)


  print("local target pose:",unpack(target_pose_local))
  print("Current stance:",unpack( util.pose_relative(uLeft_now,uRight_now) )    )
  print("footY:",2*Config.walk.footY)



  --Select initial foot based on velocity
  if target_pose_local[2]>0 then --sidestep to the left
    supportLeg = 1 --right support. left move
  else --sidestep to the right
    supportLeg = 0 --left support, right move
  end


  local pose_initial = {uTorso_now[1],uTorso_now[2],uTorso_now[3]}

  local step_queue={}

  local tSlope1 = Config.walk.tStep*Config.walk.phSingle[1]
  local tSlope2 = Config.walk.tStep*(1-Config.walk.phSingle[2])
  
  step_queue[1] = {{0,0,0},2, 0.1,1,0.1,{0,0,0},{0,0,0}}
  local step_queue_count = 1;

  local arrived = false;

  --TODO: only sidestep requires frequent pausing

--  local max_step_count = 30
  local max_step_count = Config.walk.maxStepCount or 6

  
  while step_queue_count<max_step_count and not arrived do
    step_queue_count = step_queue_count + 1    
    print("===========")
    
    local leg_movement=vector.new({0,0,0})

    initial_step, last_step = false, false
    if step_queue_count==max_step_count then last_step = true end
    if step_queue_count<=3 or step_queue_count == max_step_count-1 then
      if supportLeg ==0 then --Left support, right move
        uRight_next = util.pose_global(
          vector.new({0,-2*Config.walk.footY,0}),
          uLeft_now)
        uLeft_next = uLeft_now
        leg_movement = util.pose_relative(uRight_next,uRight_now)  
      else
        uLeft_next = util.pose_global(
          vector.new({0,2*Config.walk.footY,0}),
          uRight_now)
        uRight_next = uRight_now
        leg_movement = util.pose_relative(uLeft_next,uLeft_now)  
      end

--        if step_queue_count==2 then arrived = true end
    else
      local uTorso0 = util.pose_global({-Config.walk.supportX,0,0},uTorso_now)
      local uLeftTarget = util.pose_global({0,Config.walk.footY,0},target_pose)
      local uRightTarget = util.pose_global({0,-Config.walk.footY,0},target_pose)

      local rel_pose = util.pose_relative(target_pose,uTorso0) --global frame
      local rel_foot = util.pose_relative(uLeft_now,uRight_now)

      local rel_lfoot = util.pose_relative(uLeftTarget,uLeft_now)
      local rel_rfoot = util.pose_relative(uRightTarget,uRight_now)

      print("rel pose x:",rel_pose[1])
      print("rel lfoot x:",rel_lfoot[1],rel_lfoot[2])
      print("rel rfoot x:",rel_rfoot[1],rel_rfoot[2])
   
      local max_step_x = 0.05
      local min_step_x = -0.03


      local max_leg_movement_x = math.abs(rel_foot[1])+max_step_x
      local min_leg_movement_x = -math.abs(rel_foot[1])+min_step_x

      local max_step_y = 0.08
      local min_stance_y = 0.18 --Config.walk.stanceLimitY[1]


      if supportLeg==0 then --Left support
        leg_movement[1]  =  math.max(min_leg_movement_x,math.min(max_leg_movement_x, rel_rfoot[1]))
        leg_movement[2]  =  math.max(-max_step_y,math.min(max_step_y, rel_rfoot[2]))

      else
        leg_movement[1]  =  math.max(min_leg_movement_x,math.min(max_leg_movement_x, rel_lfoot[1]))
        leg_movement[2]  =  math.max(-max_step_y,math.min(max_step_y, rel_lfoot[2]))
      end

      if supportLeg==0 then 
        uLeft_next = uLeft_now
        uRight_next = util.pose_global(leg_movement,uRight_now)
      else
        uLeft_next = util.pose_global(leg_movement,uLeft_now)
        uRight_next = uRight_now
      end


      local uLeftRight = util.pose_relative(uLeft_next, uRight_next)      
      print("leg width:",uLeftRight[2]," limit ",min_stance_y)  
      uLeftRight[2] = math.max(min_stance_y,uLeftRight[2])
      if supportLeg==0 then --left support
        uRight_next = util.pose_global(-uLeftRight,uLeft_next)          
        leg_movement = util.pose_relative(uRight_next,uRight_now)
      else
        uLeft_next = util.pose_global(uLeftRight,uRight_next)                    
        leg_movement = util.pose_relative(uLeft_next,uLeft_now)
      end


      local rel_lfoot = util.pose_relative(uLeftTarget,uLeft_next)
      local rel_rfoot = util.pose_relative(uRightTarget,uRight_next)

      local d_lfoot = math.sqrt(rel_lfoot[1]^2+rel_lfoot[2]^2)
      local d_rfoot = math.sqrt(rel_rfoot[1]^2+rel_rfoot[2]^2)

      if d_lfoot<0.001 and d_rfoot<0.001 then
        arrived=true
      end

    end

    local uLSupport_next,uRSupport_next = step_planner.get_supports(uLeft_next,uRight_next)
    uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)

    print(unpack(leg_movement))

    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_increment(
        uLeft_next,uRight_next,uTorso_next,
        supportLeg,
        leg_movement)

    print("leg movement :x",leg_movement[1], " y",leg_movement[2])

    local new_step ={leg_movement, 
              supportLeg, 
              tSlope1, 
              Config.walk.tStep-tSlope1-tSlope2,
              tSlope2,
              {0,0,0},
              {0,Config.walk.stepHeight,0}}
    --print("t:",tSlope1,Config.walk.tStep-tSlope1-tSlope2, tSlope2 )
    step_queue[step_queue_count]=new_step

    print("uLeftnext",unpack(uLeft_next))
    print("uRightnext",unpack(uRight_next))

    print("uTOrsonext:",unpack(util.pose_relative(uTorso_next,pose_initial)))


    supportLeg = 1-supportLeg    
  end



  local pose_end = {uTorso_next[1],uTorso_next[2],uTorso_next[3]}

  print("Target pose:",unpack(target_pose_local))
  print("Actual pose:",unpack(util.pose_relative(pose_end,pose_initial)))
--    util.pose_relative(pose_end,pose_initial)    ))



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
  mcm.set_step_footholds(step_queue_vector)
  mcm.set_step_nfootholds(#step_queue)
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
   
  -- nwaypoints = hcm.get_motion_nwaypoints()
  nwaypoints = 1
  waypoints = hcm.get_motion_waypoints()

  print('# of waypoints:', nwaypoints)
  
  local raw_waypoints = vector.slice(waypoints,1,3*nwaypoints)
  print('waypoints', unpack(raw_waypoints))
  -- Check the frame of reference
  local waypoint_frame = hcm.get_motion_waypoint_frame()
	if waypoint_frame==0 then print('Waypoint Frame: LOCAL')
	else print('Waypoint Frame: GLOBAL') end

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



  local uTorso = mcm.get_status_uTorso()  
  local uTorso0 = util.pose_global({-Config.walk.supportX,0,0},uTorso)
  local target_pose_local = util.pose_relative(target_pose,uTorso0)

  if math.abs(target_pose_local[3])>10*DEG_TO_RAD then
    print("OLD STEP")
    calculate_footsteps()    
  else
    print("NEW STEP")
    calculate_footsteps_new()
  end

  motion_ch:send'preview'  
end





function state.update()
  --print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

--  if arrived and mcm.get_walk_ismoving()==0 then
  if  mcm.get_walk_ismoving()==0 then
    return 'done'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
