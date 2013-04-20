dofile('../../include.lua')
require('trajectory')
require('Kinematics')
require('Dynamics')
require('transform')
require('dcm')
require('pcm')

---------------------------------------------------------------------------
-- fsc : foot step controller
---------------------------------------------------------------------------
fsc = {}

local hip_x_state = {0, 0, 0}
local hip_y_state = {0, 0, 0}
local COG_x_state = {0, 0, 0}
local COG_y_state = {0, 0, 0}
local swing_state = {0, 0, 0}
local foot_impact_pose = {0, 0, 0, 0, 0, 0}
local foot_mid_pose = {0, 0, 0, 0, 0, 0}
local hip_x_goal = {0, 0, 0}
local hip_y_goal = {0, 0, 0}
local COG_x_goal = {0, 0, 0}
local COG_y_goal = {0, 0, 0}
local double_support = true
local single_support = false
local foot_time = 0 --goal for foot counter
local step_time = 0 --goal step time
local t = 0 --robot time
local t_step = 0 --counter for step time
local t_foot = 0 --counter for foot lift
local swing_offset = vector.zeros(6)
local torso_offset = vector.zeros(6)
local reference_foot = 'l'
  
local queue = {{0,-0.22, 2,'l'},{0, 0.22, 2, 'r'}}

 -- utilities
--------------------------------------------------------------------------------
local function calculate_COG(q)
  local torso_pose = {0, 0, 0, 0, 0, 0}
  local torso_frame = Transform.pose(torso_pose) --which frame to use?
  local COG = Dynamics.cog(q, torso_frame)
  --add foot axis fix
  return COG
end

local function calculate_COG_vel(q1, q2, dt)
  local COG1 = calculate_COG(q1)
  local COG2 = calculate_COG(q2)
  local vel = {(COG2[1] - COG[1])/dt
  vel[2] = {(COG2[2] - COG[2])/dt
  return COG1, COG2, vel
end

local function move_legs(torso, l_leg, r_leg)
--returns joint angle required to move the torso origin to given location
  --if r_leg == nil then r_leg = vector.zeros(6) end
  --if l_leg == nil then l_leg = vector.zeros(6) end
  local l_foot_frame = Transform.pose(l_leg)
  local r_foot_frame = Transform.pose(r_leg)
  local torso_frame = Transform.pose(torso)
  local qStance = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
  return qStance  
end

local function update_timers(new_time)
  local dt = new_time - t
  t_foot = t_foot + dt
  t_step = t_step + dt
end

local function foot_controller()
  --returns delta for swing foot
  local goal_pose = vector.zeros(6)
  --move foot
  if t_foot < foot_time/2 then  --raises foot
    x,xd,xdd = trajectory.minimum_jerk_step(move_foot, {1, 0, 0}, foot_time/2, t_foot)
    goal_pose = foot_mid_pose
  else --lowers foot
    x,xd,xdd = trajectory.minimum_jerk_step(move_foot, {0, 0, 0}, foot_time/2, t_foot - foot_time/2)
    goal_pose = foot_impact_pose
  end
  swing_state = vector.new{x, xd, xdd} 
  local delta = move_foot[1]*goal_pose
  return delta
end

local function compute_traj(state, goal, tau, inc)
  local x,xd,xdd = trajectory.minimum_jerk_step(hip, hip, tau, inc)
  state = vector.new{x, xd, xdd} 
  return state
end

local function ssc(tau)
  --move hip x
  local torso_delta = vector.zeros(6)
  hip_x_state = compute_traj(hip_x_state, hip_x_goal, tau, 0.004) 
  hip_y_state = compute_traj(hip_y_state, hip_y_goal, tau, 0.004)
  torso_delta[1] = hip_x_state[1]
  torso_delta[2] = hip_y_state[1]
  --move foot
  local swing_delta = foot_controller()
  if then 
    COG_x_state = compute_traj(COG_x_state, COG_x_goal, tau, 0.004)
    COG_y_state = compute_traj(COG_y_state, COG_y_goal, tau, 0.004)
  else 
    COG_des[1] = COG_des[1] + COG_x_state[2]*0.004
    COG_des[2] = COG_des[2] + COG_y_state[2]*0.004
  end
  --reassign to robot
  COG_des[1] = COG_x_state[1]
  COG_vel[1] = COG_x_state[2]
  COG_des[2] = COG_y_state[1]
  COG_vel[2] = COG_y_state[2]
  deccel_torque[1] = 8*COG_x_state[3]
  deccel_torque[2] = 8*COG_y_state[3]
  if reference_foot == 'left' then
    qt = move_legs(torso_delta + torso_offset, vector.zeros(6), swing_delta + swing_offset)
  else
    qt = move_legs(torso_delta + torso_offset, swing_delta + swing_offset, vector.zeros(6))
  end
--what to do with above variables?
end

local function dsc(tau)
  local qt = vector.zeros(12)
  local torso_delta = vector.zeros(6)
  hip_x_state = compute_traj(hip_x_state, hip_x_goal, tau, 0.004)
  hip_y_state = compute_traj(hip_y_state, hip_y_goal, tau, 0.004)
  torso_delta[1] = hip_x_state[1]
  torso_delta[2] = hip_y_state[1]
  if reference_foot == 'left' then
    qt = move_legs(torso_delta + torso_offset, vector.zeros(6), swing_offset)
  else
    qt = move_legs(torso_delta + torso_offset, swing_offset, vector.zeros(6))
  end
  COG_des = [torso_delta[1], torso_delta[2]] 
   --above approximation valid as long as desired loc. is on foot axis
  COG_vel = [0, 0]
  return qt
end

local function set_offsets()
  local torso = {}
  local swing = {}
  local l_leg = pcm:get_l_foot_pose()
  local r_leg= pcm:get_r_foot_pose()
  l_leg = Transform.pose(l_leg)
  r_leg = Transform.pose(r_leg)

  if reference_foot = 'l'
    torso = l_leg:inv()
    swing = torso*r_leg
  else
    torso = r_leg:inv()
    swing = torso*l_leg
  end
  torso_offset = torso:get_pose()
  swing_offset = swing:get_pose()
end

local function compute_ss_goals()
  local COG_x = {}
  local COG_y = {}

  hip_x_state = compute_traj(hip_x_state, hip_x_goal, step_time, 0.2*step_time)
  hip_y_state = compute_traj(hip_y_state, hip_y_goal, step_time, 0.2*step_time)
  torso_delta[1] = hip_x_state[1]
  torso_delta[2] = hip_y_state[1]
  local q1 = move_legs(torso_delta + torso_offset, vector.zeros(6), --footstep)
 
  hip_x_state = compute_traj(hip_x_state, hip_x_goal, step_time, 0.2*step_time+0.012)
  hip_y_state = compute_traj(hip_y_state, hip_y_goal, step_time, 0.2*step_time+0.012)
  torso_delta[1] = hip_x_state[1]
  torso_delta[2] = hip_y_state[1]
  local q2 = move_legs(torso_delta + torso_offset, vector.zeros(6), --footstep)

  COG_x, COG_y, COG_vel = calculate_COG_vel(q1, q2, 0.012)
  return COG_x, COG_y
end

local function new_step()
  --set new COG_x and y goals
  local footstep = queue.remove(1)
  hip_x_goal = footstep[1]
  hip_y_goal = footstep[2] - 0.020
  COG_x_goal, COG_y_goal = compute_ss_goals()
  set_offsets()
  t_step = 0
end

-- public
--------------------------------------------------------------------------------
function fsc.begin(t)
  t = 0 --walk time
  t_step = 0 --counter for step time
  t_foot = 0 --counter for foot lift
  set_offsets()
  new_step()
end

function fsc.udpate(state_est, foot_state, t)
  update_timers(t)
  if t_step > step_time then
    new_step() 
  end
  --compute footstate
  if double_support == true then
    dsc(step_time - t_step)
    local pos = math.sqrt(state_est[1][1]^2 + state_est[2][1]^2)
    local vel = math.sqrt(state_est[1][2]^2 + state_est[2][2]^2)
    if vel > pos + then--compute rule here
      single_support = true
      double_support = false
      swing_state = {0, 0, 0}
      COG_x_state = {state_est[1][1], state_est[1][2], o.hip_x_state[3]}
      COG_y_state = {state_est[2][1], state_est[2][2], o.hip_y_state[3]}
      t_foot = 0
      foot_time = --?
  elseif single_support == true then
    ssc(step_time - t_step) 
    if t_step > step_time then 
      new_step()
    elseif foot_state[11] + foot_state[5] == 2
      single_support = false
      double_support = true
    end
  end
  return --variables
end

return fsc
