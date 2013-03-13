--------------------------------------------------------
--One Legged Swing controller
--------------------------------------------------------
dofile('../../include.lua')

require('dcm')
require('pcm')
require('pid')
require('unix')
require('util')
require('Platform')
require('curses')
require('Config')
require('vector')
require('Kinematics')
require('filter')
require('Transform')

require('Proprioception')
require('Platform')
require('getch')

if not string.match(Config.platform.name, 'webots') then
  return
end
local joint = Config.joint

--------------------------------------------------------------------
-- Parameters
--------------------------------------------------------------------
--local stats = util.loop_stats(100)
local t, dt = Platform.get_time(), 0
local qt = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --desired joint angles 
local qt_comp = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --desired joint angles 
local joint_pos = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --current joint positions
local joint_vel = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --current joint velocity
local joint_acc = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local joint_pos_sense = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local raw_pos = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local joint_vel_raw = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local joint_torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --desired joint torques
local joint_torques_sense = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local pid_torques = vector.new{0, 0}
local grav_comp_torques = vector.new{0, 0} 
local ff_torques = vector.new{0, 0}
local foot_state = {[5] = 1, [6] = 1, [11] = 1, [12] = 1, ['count'] = 0} --left and right foot on ground? 
local ahrs_filt = {}
local ahrs = {}
local COP_filt = {0, 0} --filtered COP location wrt torso projection
local COG = {0, 0}
local ft_filt = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --filtered force torque data
local ft = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} -- raw force torques (debug only)
local l_leg_offset = vector.new{0, 0, 0} --xyz pos at beginning of move
local r_leg_offset = vector.new{0, 0, 0}
local delta = vector.new({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})
local joint_offset = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local state_t = 0 --keps record of time in each state
local state = 0 --keeps record of which state program is in
local run = true --boolean to quit program
local move_leg = vector.new{0, 0, 0} 
local torso_position = vector.new{0, 0, 0}
local step = 1
local printdata = false
local l_cop_t, r_cop_t = {}, {}
local bias = {0.5, 0.5} -- proportion of weight on left and right feet
local test_foot_state = {[1] = 'l', [2] = '0'}
local hip_offset = 0
local state_est_k1 = {vector.new({-0.002, 0}), vector.new({-0.146, 0})} --theta and thetadot to COG from ankle
local state_act_k1 = {vector.new({0, 0}), vector.new({0, 0})}
local pid_override = false
local step_loc_x = {.26, 52}
local step_loc_y = {0, 0}
local step_ang = 0
local walk = false
local qmid = {}
local qimp = {}
local u1 = {}

------------------------------------------------------------------
--Control objects:
------------------------------------------------------------------

local filter_b = {0.013251, 0.026502, 0.013251} --10 break freq, ts=0.004, chsi=0.7
local filter_a = {1, -1.6517, 0.70475}
local COP_filters = {filter.new(filter_b, filter_a), filter.new(filter_b, filter_a)}

local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 1 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
local pgain, igain, dgain = 300, 0, 60  --need different gains in different joints
local torque_filters = {}
for i, index in pairs(joint.index['ankles']) do
  torque_filters[index] = filter.new_second_order_low_pass(0.004, 40, 0.7) 
end

pgain, igain, dgain = 200, 120, 40
local COGx_pid = pid.new(0.004, pgain, igain, dgain)
COGx_pid:set_d_corner_frequency(20) 
local COGy_pid = pid.new(0.004, pgain, igain, dgain)
COGy_pid:set_d_corner_frequency(20)

local COG_vel_filter = {}
COG_vel_filter[1] = filter.new_differentiator(0.004, 30, 0.5)
COG_vel_filter[2] = filter.new_differentiator(0.004, 30, 0.5)
--local COG_vel_filter = filter.new_differentiator(0.004, 30, 0.5)

local pos_filters = {}
for i, index in pairs(joint.index['legs']) do
  pos_filters[index] = filter.new_second_order_low_pass(0.004, 20, 0.5)
end

local bias_filters = {}
for i, index in pairs(bias) do
  bias_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.5)
end

local vel_filters = {}
local vel_filters_raw = {}
local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 1 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
for i, index in pairs(joint.index['legs']) do
  vel_filters[index] = filter.new(filter_b, filter_a)
  vel_filters_raw[index] =  filter.new_differentiator(0.004, 5) --unfiltered
end

local acc_filters = {}
local filter_b = {6403,   -6403,   -6403,    6403} --30 freq, 3 order butter, 2 dt
local filter_a = { 1.0000,   -1.5819,    1.0147,   -0.2279}
for i, index in pairs(joint.index['legs']) do
  acc_filters[index] = filter.new(filter_b, filter_a)
end

local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 0 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
local ft_filters = {}
for i = 1, 12 do
  --ft_filters[i] = filter.new(filter_b, filter_a)
  ft_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end

local ahrs_filters = {}
for i = 1, 6 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end
for i = 7, 9 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end

------------------------------------------------------------------
--Utilities:
------------------------------------------------------------------
function compute_foot_state()
  --uses bias to set the global state defining which feet are on the ground
  local ref = 0.93
  if bias[1] <= ref and bias[2] <= ref then --check for two feet
    foot_state.count = foot_state.count + 1
    if foot_state.count >= 7 then --in dual support
      foot_state[5], foot_state[6] = 1, 1
      foot_state[11], foot_state[12] = 1, 1
    end
  elseif bias[1] > ref then
    --transfered onto left foot
    if foot_state[5] + foot_state[11] == 2 then --reset count and pid
      foot_state.count = 0 
      COGy_pid:reset()
    end
    foot_state.count = foot_state.count + 1
    foot_state[5], foot_state[6] = 1, 1
    foot_state[11], foot_state[12] = 0, 0
  elseif bias[2] > ref then
    --transfered onto right foot
    if foot_state[5] + foot_state[11] == 2 then --reset count and pid
      foot_state.count = 0 
      COGy_pid:reset()
    end
    foot_state.count = foot_state.count + 1
    foot_state.count = 0
    foot_state[5], foot_state[6] = 0, 0
    foot_state[11], foot_state[12] = 1, 1
  end
end

function COP_update() --move
  --takes force torque readings, updates left and right COP, 
  --computes robot COP, then filters and outputs
  local force_torque = dcm:get_force_torque()
  ft = force_torque --log force torque data
  l_cop = pcm:get_l_foot_cop()
  r_cop = pcm:get_r_foot_cop()
  --left_cop, right_cop = cop(force_torque) --compute COP
  --COP = robot_cop(force_torque, left_cop, right_cop) 
  COP = pcm:get_cop()
  COP_filt[1] = COP_filters[1]:update(COP[1])
  COP_filt[2] = COP_filters[2]:update(COP[2])
end

function trajectory_percentage(final_pos,time_final,time)
  --computes position, velocity and acceleration of a trajectory. 
  local velocity = 5/4*final_pos/time_final
  local accel = velocity/(time_final/5)
  local percent = 0
  local temp_accel = 0
  if (time <= time_final/5) then
    percent = 1/2*time^2*accel
    temp_accel = accel
  elseif (time <= time_final*4/5) then
    percent = 1/2*time_final/5*velocity + velocity*(time-time_final/5)
    temp_accel = 0
  elseif (time < time_final) then
    percent = 7/10*time_final*velocity + velocity*(time-time_final*4/5) - 1/2*accel*(time-4/5*time_final)^2
    temp_accel = -1*accel
  else
    percent = 1
  end
  return percent, temp_accel
end

function calculate_bias() --move
  --gives the ratio of weight on either foot
  bias[1] = 0.25*bias[1] + 0.75*ft_filt[3]/(ft_filt[3] + ft_filt[9]) --left
  bias[2] = 0.25*bias[2] + 0.75*ft_filt[9]/(ft_filt[3] + ft_filt[9]) --right
  for i = 1,2 do
    bias[i] = math.min(bias[i], 1)
    bias[i] = bias_filters[i]:update(bias[i])
  end
end

function COG_controller(des_loc) --uses loc wrt foot
  --controls the COG with the torque on the ground to maintain a 
  --COG position given by "des_loc", 
  local torques = vector.new{0, 0}
  local foot_pos = {0, 0}
  local feet_on_gnd = foot_state[5] + foot_state[11]
  if test_foot_state[1] == 'l' then --use left foot as reference
    foot_pos = pcm:get_l_foot_pose()
  else  -- use right foot as reference
    foot_pos = pcm:get_r_foot_pose()
  end
  if feet_on_gnd == 1 then
    COGx_pid:set_setpoint(des_loc[1])
    COGy_pid:set_setpoint(des_loc[2])
    torques[1] = -1*(COGy_pid.p_gain*state_est_k1[2][1] + COGy_pid.d_gain*state_est_k1[2][2]) -- COGy_pid.i_term
    torques[2] =  1*(COGx_pid.p_gain*state_est_k1[1][1] + COGx_pid.d_gain*state_est_k1[1][2]) - COGx_pid.i_term --was 0.56* for some reason
  elseif feet_on_gnd == 2 then
    COGx_pid:set_setpoint(des_loc[1])
    torques[1] = 0
    torques[2] = 1*(COGx_pid.p_gain*state_est_k1[1][1] + COGx_pid.d_gain*state_est_k1[1][2]) - COGx_pid.i_term  --was 0.56* for some reason
  end
  return torques
end

local COG_rate = 0
function generate_act_state()
  local u = vector.new({0, 0})
  local lever = vector.new({0, 0})
  local COG_rate = 0
  local foot_pos = {}
  local state_act_k = {vector.new({0, 0}),vector.new({0, 0})}
  if test_foot_state[1] == 'l' then --must extend to other foot
      foot_pos = pcm:get_l_foot_pose() else 
      foot_pos = pcm:get_r_foot_pose() end
  for i = 1, 2 do
    u[i] = pid_torques[i] + ff_torques[i] + grav_comp_torques[i]
    if i == 2 then
      u[i] = math.max(math.min(u[i],20),-20)
    else
      u[i] = -1*math.max(math.min(u[i],10),-10)
    end
    local lever = (COG[i] - foot_pos[i])
    COG_rate = COG_vel_filter[i]:update(lever)
    local theta = math.asin(lever/0.66) --state variable 1
    local thetadot = COG_rate/math.cos(theta)/0.66
    state_act_k[i] = vector.new{theta, thetadot} --estimated state, theta, thetadot
  end
  return state_act_k, u
end

function update_observer()
  --luenberg observer for the Center of gravity.
  local u = vector.new({0, 0})
  local state_est_k = {vector.new({0, 0}),vector.new({0, 0})}
  local state_act_k = {vector.new({0, 0}),vector.new({0, 0})}
  for i = 1, 2 do
    state_est_k[i] = vector.copy(state_est_k1[i]) --store past value 
  end
  state_act_k, u = generate_act_state()
  state_act_k1 = state_act_k
  u1 = u
  local akc_row_1 = vector.new{0.9075, 0.0096}
  local akc_row_2 = vector.new{-0.0096, 0.9075}
  local kc_row_1 = vector.new{0.0927, -0.0056}
  local kc_row_2 = vector.new{0.1055, 0.0927}
  --local b = vector.new{0.0000008, 0.000412}
  --local akc_row_1 = vector.new{0.8719, 0.0092}
  --local akc_row_2 = vector.new{-0.0092, 0.8719}
  --local kc_row_1 = vector.new{0.1283, -0.0052}
  --local kc_row_2 = vector.new{0.1051, 0.1283}
  local b = vector.new{0.0000016, 0.000800}
  for i = 1, 2 do
    state_est_k1[i][1] = akc_row_1*state_est_k[i] + kc_row_1*state_act_k[i] - b[1]*u[3-i]
    state_est_k1[i][2] = akc_row_2*state_est_k[i] + kc_row_2*state_act_k[i] - b[2]*u[3-i]
  end
end

function torque_components_wrt_feet_on_gnd(torques)
  --modifies torque to make appropriate for number and position of 
  --feet on the ground. If 2, then applies torque only to the direction
  --perpendicular to the axis connecting the ankle. 
  local axis = vector.new{0,0}
  local dot = 0
  local tor_comp = vector.new{0, 0}
  local feet_on_gnd = foot_state[5] + foot_state[11]
  local lf = pcm:get_l_foot_pose()
  local rf = pcm:get_r_foot_pose()
  if feet_on_gnd == 1 then
    tor_comp = vector.copy(torques)
  elseif feet_on_gnd == 2 then
    axis = vector.new{lf[1]-rf[1], lf[2]-rf[2]}
    local mag = math.sqrt(axis[1]*axis[1] + axis[2]*axis[2])
    axis = axis/mag
    dot = axis[1]*torques[1] + axis[2]*torques[2]
    tor_comp[1] = dot*axis[1]
    tor_comp[2] = dot*axis[2]
  end
  return tor_comp
end

local grav_off = false
function update_joint_torques()
  --computes gravity torque, applies bias, applies wrt ankles
  --regulates to max and min, filters, and returns result
  local torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
  calculate_bias() 
  grav_comp_torques = gravity_comp() 
  if grav_off then grav_comp_torques[2] = 0 end
  local t_x_y =  grav_comp_torques + pid_torques + ff_torques --sums torques
  t_x_y = torque_components_wrt_feet_on_gnd(t_x_y)
  torques[5] = t_x_y[2]*bias[1] 
  torques[6] = t_x_y[1]*bias[1] 
  torques[11] = t_x_y[2]*bias[2] 
  torques[12] = t_x_y[1]*bias[2] 
  torques = regulate_ankle_torques(torques) --max min
  for i, torque_loop in pairs(torque_filters) do
    --torques[i] = torque_filters[i]:update(torques[i])
  end
  return torques
end

function regulate_ankle_torques(torque) 
  --applies max and min to ankle torque to prevent feet from lifting
  --off ground.  Computes with respect to the weight of the robot
  local max = {[5] = 11, [6] = 5, [11] = 11, [12] = 5 }
  local min = {[5] = -7.5, [6] = -5, [11] = -7.5, [12] = -5 }
  local f = 0
  local temp = 0
  for i,v in pairs{[5] = 3, [6] = 3, [11] = 9, [12] = 9} do
    if v==3 then temp = bias[1] else temp = bias[2] end 
    if math.abs(ft_filt[v])>30 then f = 1 else f = 0 end
    torque[i] = math.min(torque[i], 2*f*temp*max[i])
    torque[i] = math.max(torque[i], 2*f*temp*min[i])
  end
  return torque
end

function update_joint_data()  
  --compute filters of position, velocity, and acceleration 
  raw_pos = dcm:get_joint_position_sensor('legs') --actual position
  joint_pos = dcm:get_joint_position('legs') --commanded position
  for i, filter_loop in pairs(pos_filters) do
    joint_pos_sense[i] = filter_loop:update(raw_pos[i])--position filters
  end
  for i, filter_loop in pairs(vel_filters) do
    joint_vel[i] = filter_loop:update(raw_pos[i])--velocity filters
  end    
  for i, filter_loop in pairs(vel_filters_raw) do
    joint_vel_raw[i] = filter_loop:update(raw_pos[i])--velocity filters
  end 
  for i, filter_loop in pairs(acc_filters) do
    joint_acc[i] = filter_loop:update(qt[i])--velocity filters
  end 
end

function update_force_torque() 
  --filters the force torque data for use --move
  local ft = dcm:get_force_torque() --debuggin
  for i = 1, 12 do
    ft_filt[i] = ft_filters[i]:update(ft[i])
  end 
end

function ahrs_update() --move
  ahrs = dcm:get_ahrs()
  for i = 1, 9 do
    ahrs_filt[i] = ahrs_filters[i]:update(ahrs[i])
  end 
end

function reset_states()
  local rf = pcm:get_r_foot_pose()
  state_est_k1[1][1] = math.asin((COG[1]-rf[1])/0.66);
  state_est_k1[1][2] = 0.35;
  COG_vel_filter[1]:reset()
  COG_vel_filter[1].output[1] = 0.35
end

function phase_trajectory(theta)
  local velocity = 0
  if theta>0 then --before impact
    velocity = 0.2+theta
  else --after impact
    velocity = .2
  end
  return velocity
end

function COG_walk()
  local des_vel = phase_trajectory(state_est_k1[1][1])
  print(des_vel, t)
  local vel_err = des_vel - state_est_k1[1][2]
  local vel_gain = 40
  local pos_gain = COGy_pid.p_gain
  local pos_dgain = COGy_pid.d_gain
  local torques = vector.new{0, 0}
  torques[1] = -1*(pos_gain*state_est_k1[2][1] + pos_dgain*state_est_k1[2][2])
  torques[2] = -1*vel_gain*vel_err
  return torques
end

--------------------------------------------------------------------
--Motion Functions:
--------------------------------------------------------------------

function move_legs(torso)
--returns joint angle required to move the torso origin to given location
--(negative torso z motion moves torso down)
--fix to incorporate foot pos wrt torso coordinate center 
  local x_offset = l_leg_offset[1]
  local y_offset = l_leg_offset[2]
  local z_offset = l_leg_offset[3]
  local l_foot_frame = Transform.pose({x_offset-torso[1], y_offset-torso[2], z_offset-torso[3], 0, 0, 0})
  local r_foot_frame = Transform.pose({x_offset-torso[1],-y_offset-torso[2], z_offset-torso[3], 0, 0, 0})
  local torso_frame = Transform.pose({0, 0, 0, 0, 0, 0})
  local qStance = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
  return qStance  
end

function move_r_leg(r_leg)
  local l_foot_frame = Transform.pose({l_leg_offset[1], l_leg_offset[2], l_leg_offset[3], 0, 0, 0})
  local r_foot_frame = Transform.pose({r_leg_offset[1] + r_leg[1], r_leg_offset[2] + r_leg[2], r_leg_offset[3] + r_leg[3], 0, 0, 0} )
  local torso_frame = Transform.pose({0, 0, 0, 0, 0, 0})
  local qLeg = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
  return qLeg
end

function sagital_balance(offset) --controls pitch axes
  local gains = vector.new{337.7, -58.8, 13.5, -2.7}
  local states = vector.new{joint_pos[5]-offset[1], joint_pos[3]-offset[2], joint_vel[5], joint_vel[3]}
  local acc = gains*states
  return acc
end

function swing_balance()
  --computes feedforward torques to apply to ankle during swing phase
  local C_alpha = 1--1.352 --change this term to minimize fx
  local p_fx = .2
  ff_torques[2] = -1*C_alpha*joint_acc[9] --+ ft_filt[1]*p_fx
  ff_torques[1] = 0
end

function gravity_comp()
  --computes torque at ankle required to offset gravity 
  local mg = 220
  local foot_pos = {}
  if test_foot_state[1] == 'l' then 
    foot_pos = pcm:get_l_foot_pose()
  else 
    foot_pos = pcm:get_r_foot_pose()
  end
  local ty = (COG[1] - foot_pos[1])*mg 
  local tx = -1*(COG[2] - foot_pos[2])*mg 
  return vector.new{tx, ty}
end

local hip_filter = filter.new_second_order_low_pass(0.004, 20, 0.7)
function orient_torso(des_pos)
  qt_comp = vector.copy(qt)
  local max_vel = 1
  local gain = -.15
  local temp = hip_filter:update(gain*(des_pos - ahrs_filt[8]))
  --hip_offset = hip_offset + gain*(des_pos - ahrs_filt[8])
  hip_offset = hip_offset + temp
  --qt_comp[3] = qt[3] + hip_offset
  --qt_comp[9] = qt[9] + hip_offset
end

function find_stride_joint_angles(r, theta)
  local qimpact = {}
  local qmidstride = {}
  local stance = Transform.pose({0, 0, 0, 0, 0, 0})
  local swing = Transform.pose({r, 0, 0, 0, 0, 0} )
  local torso_pos = Transform.pose({r/2, 0, 0.746, 0, 0, 0})
  if test_foot_state[1] == 'l' then 
    qimpact = Kinematics.inverse_pos_legs(stance, swing, torso_pos)
  else 
    qimpact = Kinematics.inverse_pos_legs(swing, stance, torso_pos)
  end
  local stance = Transform.pose({0, 0, 0, 0, 0, 0})
  local swing = Transform.pose({0,0, 0.05, 0, 0, 0})
  local torso_pos = Transform.pose({0, 0, 0.756, 0, 0, 0})
  if test_foot_state[1] == 'l' then 
    qmidstride = Kinematics.inverse_pos_legs(stance, swing, torso_pos)
  else 
    qmidstride = Kinematics.inverse_pos_legs(swing, stance, torso_pos)
  end
  return qimpact, qmidstride
end

function stride_percent_joint_angles()
  
end 

------------------------------------------------------------------------
--State machine
------------------------------------------------------------------------
function state_machine(t) 
  if (state == 0) then
    l_leg_offset = pcm:get_l_foot_pose()
    torso = vector.new{0, 0, -0.05, 0, 0, 0} 
    joint_offset = move_legs(torso)
    joint_offset = vector.new(joint_offset)
    if state_t >= .5 then
      print('ident_stride', ident)
      print("move to ready", t)
      state = 1
      state_t = 0
    end
  elseif (state == 1) then --move to ready position
    local percent = trajectory_percentage(1, 2, state_t)
    qt = joint_offset*percent
    if (percent >= 1) then 
      state = 2
      state_t = 0
      print("wait for 0.5", t)
    end
  elseif (state == 2) then --wait specified time
    if (state_t > .5) then  
      print(t)
      state = 3
      state_t = 0
    end
  elseif (state == 3) then --measure COG
    local left_foot_xy = pcm:get_l_foot_pose()
    COG_shift = {left_foot_xy[1] - COP_filt[1], left_foot_xy[2] - COP_filt[2], 0}
    torso = vector.new{1.182*COG_shift[1], 1.2*COG_shift[2], 0}
    l_leg_offset = pcm:get_l_foot_pose()
    state = 4
    state_t = 0
    print("move over COG", t)
  elseif (state == 4) then     --move to over COG
    if state_t <= 0.005 then hip_offset = 0 end
    local percent = trajectory_percentage(1, 2, state_t)
    qt = move_legs(percent*torso)
    if (percent >= 1) then  
      state = 5
      state_t = 0
      print("wait 0.7", t)
    end
  elseif (state == 5) then --pause for 0.7
    if (state_t > 1.7) then
      state = 6
      state_t = 0
    end
  elseif (state == 6) then -- prepare to lift right foot
    l_leg_offset = pcm:get_l_foot_pose()  
    r_leg_offset = pcm:get_r_foot_pose()  
    move_leg =vector.new{0, 0.02, 0.05}
    state = 7
    state_t = 0
    print("lift leg", t)
  elseif (state == 7) then --lift right leg
    if state_t <= 0.005 then hip_offset = 0 end
    local percent = trajectory_percentage(1, 1, state_t)
    util.ptable(r_leg_offset)
    qt = move_r_leg(percent*move_leg)
    if (percent >= 1) then
      state = 8
      state_t = 0
      print("wait .5", t)
      joint_offset = vector.copy(joint_pos)
    end
  elseif (state == 8) then --compute walking joint angles
    if state_t>=0.5 then
      qimp, qmid = find_stride_joint_angles(0.295, 0.25)
      joint_offset = vector.copy(qt)
      state = 9
      state_t = 0
      print("begin stride", t)
      delta = qimp - joint_offset
    end
  elseif (state == 9) then --begin walk @ second half of step
    walk = true
    local percent = trajectory_percentage(1, 0.9, state_t)
    qt = percent*delta + joint_offset
    if foot_state[11] == 1 then  --right foot impact
      print("recover attempt", t)
      state = 10
      state_t = 0
      test_foot_state[1] = 'r'
      joint_offset = vector.copy(qt) 
      qimp, qmid = find_stride_joint_angles(0.295, 0.25)
      delta = qmid - joint_offset 
      reset_states()
    end
  elseif (state == 10) then --first half of step
    local percent = trajectory_percentage(1, 0.8, state_t)
    qt = percent*delta + joint_offset
    if percent >= 1 then
      joint_offset = vector.copy(qt) --prepares for move 
      delta = qimp - joint_offset 
      state = 11
      state_t = 0
    end
  elseif (state == 11) then --second half of step
    local percent = trajectory_percentage(1, 0.8, state_t) 
    qt = percent*delta + joint_offset
    if foot_state[5] == 1 then --left foot impact
      state = 12
      state_t = 0
      test_foot_state[1] = 'l'
      joint_offset = vector.copy(qt) --prepares for move 
      qimp, qmid = find_stride_joint_angles(0.295, 0.25)
      delta = qmid - joint_offset --sets delta for move
      reset_states()
    end
  elseif (state == 12) then --first half of step
    local percent = trajectory_percentage(1, 0.8, state_t)
    qt = percent*delta + joint_offset
    if percent >= 1 then
      joint_offset = vector.copy(qt) --prepares for move 
      delta = qimp - joint_offset 
      state = 13
      state_t = 0
    end
  elseif (state == 13) then --second half of step
    local percent = trajectory_percentage(1, 0.8, state_t) 
    qt = percent*delta + joint_offset
    if foot_state[11] == 1 then --left foot impact
      run = false
      state = 12
      state_t = 0
      test_foot_state[1] = 'r'
      joint_offset = vector.copy(qt) --prepares for move 
      qimp, qmid = find_stride_joint_angles(0.295, 0.25)
      delta = qmid - joint_offset --sets delta for move
      reset_states()
    end
  end
end

--------------------------------------------------------------------
--Initialize
--------------------------------------------------------------------
unix.usleep(5e5)
Platform.entry()
Proprioception.entry()
print('after entry')
dcm:set_joint_enable(0,'all')
local set_values = dcm:get_joint_position('legs') --records original joint pos
dcm:set_joint_stiffness(1, 'all') -- position control
dcm:set_joint_stiffness(0, 'ankles')
dcm:set_joint_damping(0, 'ankles')
dcm:set_joint_force({0, 0, 0, 0},'ankles')
dcm:set_joint_enable(1, 'all')
qt = vector.copy(set_values) 
printdata = true

local ident = "t1"
local fw_bias = assert(io.open("Logs/fw_bias"..ident..".txt","w"))
local fw_joint_pos = assert(io.open("Logs/fw_joint_pos"..ident..".txt","w"))
local fw_grav_comp_torques = assert(io.open("Logs/fw_grav_comp_torques"..ident..".txt","w"))
local fw_qt = assert(io.open("Logs/fw_qt"..ident..".txt","w"))
local fw_qt_comp = assert(io.open("Logs/fw_qt_comp"..ident..".txt","w"))
local fw_raw_pos = assert(io.open("Logs/fw_raw_pos"..ident..".txt","w"))
local fw_joint_pos_sense = assert(io.open("Logs/fw_joint_pos_sense"..ident..".txt","w"))
local fw_joint_torques_sense = assert(io.open("Logs/fw_joint_torques_sense"..ident..".txt","w"))
local fw_joint_vel_raw = assert(io.open("Logs/fw_joint_vel_raw"..ident..".txt","w"))
local fw_joint_vel = assert(io.open("Logs/fw_joint_vel"..ident..".txt","w"))
local fw_joint_acc = assert(io.open("Logs/fw_joint_acc"..ident..".txt","w"))
local fw_joint_torques = assert(io.open("Logs/fw_joint_torques"..ident..".txt","w"))
local fw_ff_torques = assert(io.open("Logs/fw_ff_torques"..ident..".txt","w"))
local fw_pid_torques = assert(io.open("Logs/fw_pid_torques"..ident..".txt","w"))
local fw_joint_force = assert(io.open("Logs/fw_joint_force"..ident..".txt","w"))
local fw_joint_force_sense = assert(io.open("Logs/fw_joint_force_sense"..ident..".txt","w"))
local fw_COG = assert(io.open("Logs/fw_COG"..ident..".txt","w"))
local fw_ft_filt = assert(io.open("Logs/fw_ft_filt"..ident..".txt","w"))
local fw_ft = assert(io.open("Logs/fw_ft"..ident..".txt","w"))
local fw_lr_cop_t = assert(io.open("Logs/fw_lr_cop_t"..ident..".txt","w"))
local fw_COP_filt = assert(io.open("Logs/fw_COP_filt"..ident..".txt","w"))
local fw_wrench = assert(io.open("Logs/fw_wrench"..ident..".txt","w"))
local fw_ahrs_filt = assert(io.open("Logs/fw_ahrs_filt"..ident..".txt","w"))
local fw_ahrs = assert(io.open("Logs/fw_ahrs"..ident..".txt","w"))
local fw_lf = assert(io.open("Logs/fw_lf"..ident..".txt","w"))
local fw_rf = assert(io.open("Logs/fw_rf"..ident..".txt","w"))
local fw_pcm_cog = assert(io.open("Logs/fw_pcm_cog"..ident..".txt","w"))
local fw_hip_offset = assert(io.open("Logs/fw_hip_offset"..ident..".txt","w"))
local fw_foot_state = assert(io.open("Logs/fw_foot_state"..ident..".txt","w"))
local fw_pid_terms = assert(io.open("Logs/fw_pid_terms"..ident..".txt","w"))
local fw_state_est_k1 = assert(io.open("Logs/fw_state_est_k1"..ident..".txt","w"))
local fw_state_act_k1 = assert(io.open("Logs/fw_state_act_k1"..ident..".txt","w"))
local fw_u1 = assert(io.open("Logs/fw_u1"..ident..".txt","w"))

function write_to_file(filename, data, test)
  for i = 1,#data do
    filename:write(data[i], ", ")
  end
  filename:write(t, "\n")
end

--------------------------------------------------------------------
--Main
--------------------------------------------------------------------
while run do 
  Platform.update()
  Proprioception.update()
  dt = Platform.get_time() - t --
  t = t + dt --simulation time
  state_t = state_t + dt --time used in state machine
  step = step + 1  --step number
--sensor updates
  update_joint_data()
  ahrs_update()
  update_force_torque()
  COP_update()
  compute_foot_state()
  local joint_pos_COG = vector.copy(joint_pos) --
  joint_pos_COG[5] = joint_pos_sense[5]
  joint_pos_COG[6] = joint_pos_sense[6]
  joint_pos_COG[11] = joint_pos_sense[11]
  joint_pos_COG[12] = joint_pos_sense[12]
  COG = pcm:get_cog()--COG_update(joint_pos_COG)
  state_machine(t)
  if state >= 1 then orient_torso(0) end--modify qt's to upright torso
  if walk == true then
    pid_torques = COG_walk()
  else 
    pid_torques = COG_controller({0.0, 0})
  end
  if pid_override then  pid_torques[2] = 0 end --, pid_torques[1] = 0,
  joint_torques = update_joint_torques()
  update_observer()
  joint_torques_sense = dcm:get_joint_force_sensor('legs')
--implement actions
  dcm:set_joint_position(qt_comp, 'legs')  
  dcm:set_joint_force(joint_torques, 'legs')  

  if (printdata) then -- mod_print == 0 and
    write_to_file(fw_bias, bias)
    write_to_file(fw_grav_comp_torques, grav_comp_torques)
    write_to_file(fw_joint_pos, joint_pos)
    write_to_file(fw_joint_pos_sense, joint_pos_sense)
    write_to_file(fw_joint_torques_sense, joint_torques_sense)
    write_to_file(fw_raw_pos, raw_pos)
    write_to_file(fw_joint_vel_raw, joint_vel_raw)
    write_to_file(fw_joint_vel, joint_vel)
    write_to_file(fw_joint_acc, joint_acc)
    write_to_file(fw_qt, qt)
    write_to_file(fw_qt_comp, qt_comp)
    write_to_file(fw_joint_torques, joint_torques)
    write_to_file(fw_ff_torques, ff_torques)
    write_to_file(fw_pid_torques, {pid_torques[1], pid_torques[2]})
    write_to_file(fw_joint_force, dcm:get_joint_force())
    write_to_file(fw_joint_force_sense, dcm:get_joint_force_sensor())
    write_to_file(fw_COG, COG)
    write_to_file(fw_ft_filt, ft_filt)
    write_to_file(fw_ft, ft)
    write_to_file(fw_lr_cop_t, {l_cop_t[1], l_cop_t[2], r_cop_t[1], r_cop_t[2]})
    write_to_file(fw_COP_filt, COP_filt)
    write_to_file(fw_wrench, wrench)
    write_to_file(fw_ahrs_filt, ahrs_filt)
    write_to_file(fw_ahrs, ahrs)
    write_to_file(fw_lf, pcm:get_l_foot_pose())
    write_to_file(fw_rf, pcm:get_r_foot_pose())
    write_to_file(fw_pcm_cog, pcm:get_cog())
    write_to_file(fw_hip_offset, {hip_offset})
    write_to_file(fw_state_est_k1, {state_est_k1[1][1],state_est_k1[1][2],state_est_k1[2][1],state_est_k1[2][2]})
    write_to_file(fw_state_act_k1, {state_act_k1[1][1],state_act_k1[1][2],state_act_k1[2][1],state_act_k1[2][2]})
    write_to_file(fw_u1,u1)
    write_to_file(fw_foot_state, {foot_state[5],foot_state[11]})
    write_to_file(fw_pid_terms, {COGx_pid.p_term, COGx_pid.i_term, COGx_pid.d_term, COG_rate, t})
  end
end
Platform.exit()


