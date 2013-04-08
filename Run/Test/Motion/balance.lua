--------------------------------------------------------
--runs balance but just outputs torques, does not send to joints
--used to check COG position and 
--------------------------------------------------------
dofile('../../include.lua')
require('dcm')
require('pcm')
require('pid')
require('unix')
require('util')
require('Platform')
require('Config')
require('vector')
require('Kinematics')
require('filter')
require('Transform')
require('Proprioception')
require('Platform')
require('getch')
require('trajectory')
local joint = Config.joint

--------------------------------------------------------------------
-- Parameters
--------------------------------------------------------------------
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
local COG_des = {0, 0}
local ft_filt = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --filtered force torque data
local ft = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} -- raw force torques (debug only)
local lf = {}
local rf = {}
local l_leg_offset = vector.new{0, 0, 0} --xyz pos at beginning of move
local r_leg_offset = vector.new{0, 0, 0}
local joint_offset = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local state_t = 0 --keps record of time in each state
local state = 0 --keeps record of which state program is in
local run = true --boolean to quit program
local torso_b = vector.zeros(6)
local delta = vector.zeros(6)
local step = 1
local printdata = false
local l_cop_t, r_cop_t = {}, {}
local bias = {0.5, 0.5} -- proportion of weight on left and right feet
local test_foot_state = {[1] = 'l', [2] = '0'}
local state_est_k1 = {vector.new({-0.002, 0}), vector.new({-0.146, 0})} 
local state_est_d1 = vector.new({-0.002, 0})
local qmid = {}
local qimp = {}
local data = {}
local double_support = false
local axis = vector.new{0, 0}
local ds_lever = 0
local hip_state = {0,0,0,0}

local goal = 0
local error_cap = {0,0,0}
local state_step = 0
------------------------------------------------------------------
--Control objects:
------------------------------------------------------------------
local COP_filters = {filter.new_second_order_low_pass(0.004, 20, 0.7), filter.new_second_order_low_pass(0.004, 20, 0.7)}

local torque_filters = {}
for i, index in pairs(joint.ankles) do
  torque_filters[index] = filter.new_low_pass(0.004,30) 
end

pgain, igain, dgain = 120, 40, 45
local COGx_pid = pid.new(0.004, pgain, igain, dgain)
COGx_pid:set_d_corner_frequency(20) 
pgain, igain, dgain = 120, 30, 30
local COGy_pid = pid.new(0.004, pgain, igain, dgain)
COGy_pid:set_d_corner_frequency(20)
pgain, igain, dgain = 100, 30, 30
local COGd_pid = pid.new(0.004, pgain, igain, dgain)
COGd_pid:set_d_corner_frequency(20)

local COG_vel_filter = {}
COG_vel_filter[1] = filter.new_differentiator(0.004, 30, 0.5)
COG_vel_filter[2] = filter.new_differentiator(0.004, 30, 0.5)
COG_vel_filter_d = filter.new_differentiator(0.004, 30, 0.5)

local pos_filters = {}
for i, index in pairs(joint.legs) do
  pos_filters[index] = filter.new_second_order_low_pass(0.004, 60, 0.5)
end

local bias_filters = {}
for i, index in pairs(bias) do
  bias_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.5)
end

local vel_filters = {}
local vel_filters_raw = {}
local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 1 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
for i, index in pairs(joint.legs) do
  vel_filters[index] = filter.new(filter_b, filter_a)
  vel_filters_raw[index] =  filter.new_differentiator(0.004, 5) --unfiltered
end

local acc_filters = {}
local filter_b = {6403,   -6403,   -6403,    6403} --30 freq, 3 order butter, 2 dt
local filter_a = { 1.0000,   -1.5819,    1.0147,   -0.2279}
for i, index in pairs(joint.legs) do
  acc_filters[index] = filter.new(filter_b, filter_a)
end

local ft_filters = {}
for i = 1, 12 do
  ft_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end

local ahrs_filters = {}
for i = 1, 6 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
end
for i = 7, 9 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
end

local lf_filters = {}
local rf_filters = {}
for i = 1, 6 do
  lf_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
  rf_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
end

local COG_filters = {}
for i = 1, 3 do
  COG_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
end
------------------------------------------------------------------
--Utilities:
------------------------------------------------------------------
function compute_foot_state()
  --uses bias to set the global state defining which feet are on the ground
  local ref = 0.95
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

function foot_pose_update()
  local lf_temp = pcm:get_l_foot_pose()
  local rf_temp = pcm:get_r_foot_pose()
  for i = 1,6 do
    rf[i] = rf_filters[i]:update(rf_temp[i])
    lf[i] = lf_filters[i]:update(lf_temp[i])
  end
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
  if double_support == true then feet_on_gnd = 2 end
  if test_foot_state[1] == 'l' then --use left foot as reference
    foot_pos = vector.copy(lf)
  else  -- use right foot as reference
    foot_pos = vector.copy(rf)
  end
  if feet_on_gnd == 1 then
    COGx_pid:set_setpoint(des_loc[1])
    COGx_pid:update(state_est_k1[1][1])
    COGy_pid:set_setpoint(des_loc[2])
    COGy_pid:update(state_est_k1[2][1])

    torques[1] = -1*COGy_pid.p_gain*(state_est_k1[2][1] - des_loc[2]) 
    torques[1] = torques[1] - COGy_pid.d_gain*state_est_k1[2][2] -- COGy_pid.i_term
    torques[2] = COGx_pid.p_gain*(state_est_k1[1][1] - des_loc[1]) 
    torques[2] = torques[2] + COGx_pid.d_gain*state_est_k1[1][2] - COGx_pid.i_term 
  elseif feet_on_gnd == 2 then
    COGd_pid:set_setpoint(0)
    COGd_pid:update(state_est_d1[1])
    local sum_torque = COGx_pid.p_gain*state_est_d1[1] 
    sum_torque =  sum_torque + COGx_pid.d_gain*state_est_d1[2] - COGd_pid.i_term 
    torques = axis*sum_torque
  end
  return torques
end

function generate_act_state()
  local u = vector.new({0, 0})
  local lever = vector.new({0, 0})
  local COG_rate = 0
  local foot_pos = {}
  local state_act_k = {vector.new({0, 0}),vector.new({0, 0})}
  if test_foot_state[1] == 'l' then 
      foot_pos = vector.copy(lf) else 
      foot_pos = vector.copy(rf) end
  for i = 1, 2 do
    u[i] = pid_torques[i] + ff_torques[i] + grav_comp_torques[i]
    --consider using joint_torques here instead of u
    if i == 2 then
      u[i] = math.max(math.min(u[i],22),-15)
    else
      u[i] = -1*math.max(math.min(u[i],10),-10)
    end
  --  if i==1 then print('act_lever', lever) end
    local lever = (COG[i] - foot_pos[i])
    COG_rate = COG_vel_filter[i]:update(lever)
    local theta = math.asin(lever/0.66) --state variable 1
    local thetadot = COG_rate/math.cos(theta)/0.66 --state variable 2
    state_act_k[i] = vector.new{theta, thetadot} 
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

function generate_act_state_double_support()
  local u = vector.new({0, 0})
  local COG_rate = 0
  local foot_pos = {}
  local state_act_d = vector.new({0, 0})
  update_foot_axis()
  for i = 1, 2 do
    u[i] = pid_torques[i] + ff_torques[i] + grav_comp_torques[i]
    if i == 2 then u[i] = math.max(math.min(u[i],22),-15)
    else u[i] = -1*math.max(math.min(u[i],10),-10) end
  end
  u = axis*u
  local A = axis[1]/axis[2]
  local C = lf[1] - A*lf[2]
  ds_lever = -1*(A*COG[2] - 1*COG[1] + C)/math.sqrt(A*A + 1)
  COG_rate = COG_vel_filter_d:update(ds_lever)
  local theta = math.asin(ds_lever/0.66) --state variable 1
  local thetadot = COG_rate/math.cos(theta)/0.66 --state variable 2
  state_act_d = vector.new{theta, thetadot} 
  return state_act_d, u
end

function update_observer_double_support()
  --luenberg observer for double support
  local u = 0
  local state_est_d = vector.new{0, 0}
  local state_act_d = vector.new{0, 0}
  state_est_d = vector.copy(state_est_d1) --store past est value 
  state_act_d, u = generate_act_state_double_support()
  state_act_d1 = state_act_d --store past actual values
  local akc_row_1 = vector.new{0.9075, 0.0096}
  local akc_row_2 = vector.new{-0.0096, 0.9075}
  local kc_row_1 = vector.new{0.0927, -0.0056}
  local kc_row_2 = vector.new{0.1055, 0.0927}
  local b = vector.new{0.0000016, 0.000800}
  for i = 1, 2 do
    state_est_d1[1] = akc_row_1*state_est_d + kc_row_1*state_act_d - b[1]*u
    state_est_d1[2] = akc_row_2*state_est_d + kc_row_2*state_act_d - b[2]*u
  end
end

function torque_components_wrt_feet_on_gnd(torques)
  --modifies torque to make appropriate for number and position of 
  --feet on the ground. If 2, then applies torque only to the direction
  --perpendicular to the axis connecting the ankle. 
  local dot = 0
  local tor_comp = vector.new{0, 0}
  local feet_on_gnd = foot_state[5] + foot_state[11]
  if feet_on_gnd == 1 then  
    tor_comp = vector.copy(torques)
  elseif feet_on_gnd == 2 then
    dot = axis[1]*torques[1] + axis[2]*torques[2]
    tor_comp[1] = dot*axis[1]
    tor_comp[2] = dot*axis[2]
  end
  return tor_comp
end

function update_foot_axis()
  axis = vector.new{lf[1]-rf[1], lf[2]-rf[2]}
  local mag = math.sqrt(axis[1]*axis[1] + axis[2]*axis[2])
  axis = axis/mag
end

function update_joint_torques()
  --computes gravity torque, applies bias, applies wrt ankles
  --regulates to max and min, filters, and returns result
  local torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
  calculate_bias() 
  grav_comp_torques = gravity_comp() 
  local t_x_y =  grav_comp_torques + pid_torques + ff_torques 
  t_x_y = torque_components_wrt_feet_on_gnd(t_x_y)--apply across foot axis
  torques[5] = t_x_y[2]*bias[1] 
  torques[6] = t_x_y[1]*bias[1] 
  torques[11] = t_x_y[2]*bias[2] 
  torques[12] = t_x_y[1]*bias[2] 
  torques = regulate_ankle_torques(torques) --max min
  for i, torque_loop in pairs(torque_filters) do
    torques[i] = torque_filters[i]:update(torques[i])
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
  --note: turned some filters off
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

function COG_update()
  --returns location of COG wrt base frame
  local COG_temp = pcm:get_cog()  --used to be COG_temp
--  for i = 1, 3 do
  --  COG[i] = COG_filters[i]:update(COG_temp[i])
 -- end 
  local pos = vector.copy(raw_pos)
  local jnt_vec_x = vector.new{0, 0, 0, 1}
  local jnt_vec_y = vector.new{0, 0, 1}
--  local bsx = vector.new{-0.2484, -0.0554, -0.0196, 0.0080}
--  local bsy = vector.new{0.2108, 0.0390, -0.0018}
  local bsx = vector.new{-0.2649, -0.0608, -0.0210, 0.0050}
  local bsy = vector.new{0.2285, 0.0460, -0.0007}  
  local correction = {}
  jnt_vec_x[1] = ahrs_filt[8]
  jnt_vec_x[2] = pos[3] +  pos[9]
  jnt_vec_x[3] = pos[4] + pos[10] 
  jnt_vec_y[1] = ahrs_filt[7]
  jnt_vec_y[2] = pos[2] + pos[8]
  correction[1] = jnt_vec_x*bsx
  correction[2] = jnt_vec_y*bsy
  COG[1] = COG_temp[1] + correction[1]
  COG[2] = COG_temp[2] + correction[2]
  COG[3] = COG_temp[3]
  --COG = pcm:get_cog()  ------ webots only
end

function ahrs_update() --move
  ahrs = dcm:get_ahrs()
  for i = 1, 9 do
    ahrs_filt[i] = ahrs_filters[i]:update(ahrs[i])
  end 
end

function reset_states() 
  local foot_pose = {}
  if test_foot_state[1] == 'r' then
    foot_pose = pcm:get_r_foot_pose()
    state_est_k1[1][1] = math.asin((COG[1]-rf[1])/0.66)
  else
    foot_pose = pcm:get_l_foot_pose()
    state_est_k1[1][1] = math.asin((COG[1]-rf[1])/0.66)
  end
    state_est_k1[1][2] = 0.1;
    COG_vel_filter[1]:reset()
    COG_vel_filter[1].output[1] = 0.1
end

--------------------------------------------------------------------
--Motion Functions:
--------------------------------------------------------------------
function move_legs(torso)
--returns joint angle required to move the torso origin to given location
  local l_foot_frame = Transform.pose(l_leg_offset)
  local r_foot_frame = Transform.pose(r_leg_offset)
  local torso_frame = Transform.pose(torso)
  local qStance = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
  return qStance  
end

function find_stride_joint_angles(r, y)
  local qimpact = {}
  local qmidstride = {}
  local stance = Transform.pose({0, 0, 0, 0, 0, 0})
  local swing = {}
  local torso_pos = {}
  if test_foot_state[1] == 'l' then 
    swing = Transform.pose({r, -y, 0, 0, 0, 0} )
    torso_pos = Transform.pose({r/4, -y/2, 0.746, 0, 0, 0})
    qimpact = Kinematics.inverse_pos_legs(stance, swing, torso_pos)
  else 
    swing = Transform.pose({r, y, 0, 0, 0, 0} )
    torso_pos = Transform.pose({r/4, y/2, 0.746, 0, 0, 0})
    qimpact = Kinematics.inverse_pos_legs(swing, stance, torso_pos)
  end
  if test_foot_state[1] == 'l' then 
    swing = Transform.pose({0, -y, 0.05, 0, 0, 0})
    torso_pos = Transform.pose({0, 0, 0.756, 0, 0, 0})
    qmidstride = Kinematics.inverse_pos_legs(stance, swing, torso_pos)
  else 
    swing = Transform.pose({0, y, 0.05, 0, 0, 0})
    torso_pos = Transform.pose({0, 0, 0.756, 0, 0, 0})
    qmidstride = Kinematics.inverse_pos_legs(swing, stance, torso_pos)
  end
  return qimpact, qmidstride
end

function instep_stabilizer()
  local qStance = {}
  local l_foot_pose = vector.copy(l_foot_offset)
  local r_foot_pose = vector.copy(r_foot_offset)
  
  local u = 0.001*state_est_k1[2][2]
  l_foot_pose[3] = l_foot_pose[3] + u
  r_foot_pose[3] = r_foot_pose[3] - u

  local l_foot_frame = Transform.pose(l_foot_pose)
  local r_foot_frame = Transform.pose(r_foot_pose)

  qStance = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_b)
  return qStance
end

function gravity_comp()
  local mg = 190
  local foot_pos = {}
  local tx, ty = 0, 0
  local feet_on_gnd = foot_state[5] + foot_state[11]
  if double_support == true then feet_on_gnd = 2 end
  if test_foot_state[1] == 'l' then
    foot_pos = pcm:get_l_foot_pose()
  else
    foot_pos = pcm:get_r_foot_pose()
  end
  if feet_on_gnd == 1 then 
    tx = -1*(COG[2] - foot_pos[2])*mg
    ty = (COG[1] - foot_pos[1])*mg
  else
    tx = axis[1]*ds_lever*mg
    ty = axis[2]*ds_lever*mg
   -- print('axis ty', ty, ds_lever, (COG[1] - foot_pos[1])*mg)
  end
  return vector.new{tx, ty}
end

function sagital_balance(offset) --controls pitch axes
  local gains = vector.new{337.7, -58.8, 13.5, -2.7}
  local states = vector.new{joint_pos[5]-offset[1], joint_pos[3]-offset[2], joint_vel[5], joint_vel[3]}
  local acc = gains*states
  return acc
end

------------------------------------------------------------------------
--State machine
------------------------------------------------------------------------
local storemove = {0,0,0,0}
local storemove2 = {0,0,0,0}
local qt_test = vector.zeros(12)
local ratio = 1
function state_machine(t) 
  if (state == 0) then
    printdata = true
    if state_t >=  1 then
      print('state_t', state_t)
      l_leg_offset = vector.copy(lf)
      r_leg_offset = vector.copy(rf)
      joint_offset = move_legs(vector.new{0, 0, -0.05, 0, 0, 0})
      joint_offset = vector.new(joint_offset)
      state = 1
      state_t = 0
      --run = false
    end
  elseif (state == 1) then --move to ready position
    local percent = trajectory_percentage(1, 1.5, state_t)
    qt = joint_offset*percent
    if state_t < 1.3 then
      ff_torques = vector.new{0,-5}
      COGd_pid.d_gain = 10
    else
      ff_torques = vector.new{0, 0}
      COGd_pid.d_gain = 30
    end
    if (percent >= 1) then 
      ff_torques = vector.new{0, 0}
      state = 2
      state_t = 0
      print("wait for 0.5", t)
      --run = false
    end
  elseif (state == 2) then --wait 
    if (state_t > 2) then  
      print(t)
      state = 3
      state_t = 0
      --run = false
      l_leg_offset = vector.copy(lf)
      r_leg_offset = vector.copy(rf)
      trav_offset = lf[2] - COG[2]
      delta = vector.new{0,0,0,ahrs_filt[7], ahrs_filt[8],0}
      goal = (lf[2] - COG[2]) - 0.03
      hip_state = {0,0,0}
    end
  elseif (state == 3) then  --move COG
    local tau = 2 - state_t --remaining time
    if state_t > 0.4 then 
      ratio = (trav_offset - (lf[2] - COG[2]))/(l_leg_offset[2] - lf[2])
      goal = 0.085 / ratio
    end
    if state_t > 1.5 then ff_torques[1] = -4 end
    local x,xd,xdd = trajectory.minimum_jerk_step(hip_state, {goal, 0, 0}, tau, 0.004)
    hip_state = vector.new{x, xd, xdd} 
    delta[2] = hip_state[1]
    qt = move_legs(delta)
    if (state_t >2) then  -- true then --
      --run = false
      ff_torques[1] = 0
      state = 4
      state_t = 0
      qimp, qmid = find_stride_joint_angles(0.25, 0.15)
      joint_offset = vector.copy(qt)
      delta = qmid - joint_offset
    end
  elseif (state == 4) then --lift leg
    local percent = trajectory_percentage(1, 2, state_t)
    qt = delta*percent + joint_offset
    if percent>=1 then
      --run = false
      print('take step', t)
      state = 5
      state_t = 0
      joint_offset = vector.copy(qt)
      delta = qimp - joint_offset
    end
  elseif (state == 5) then
    if (state_t > 2) then
      run = false
    end
  elseif (state == 5) then --take step
    local percent = trajectory_percentage(1, 1.5, state_t)
    qt = delta*percent + joint_offset
    COG_des[1] = 0.16*state_t/1.6
    COG_des[2] = -0.06*state_t/1.6
    if foot_state[11] == 1 then  --right foot impact
      print("impact t cog s_t", t, COG_des[1], COG_des[2], state_t)
      --run = false
      double_support = true
      state = 6
      state_t = 0
      test_foot_state[1] = 'r'
      reset_states()
      hip_state[1] = {0, state_est_k1[1][2]*0.66, 0}
      hip_state[2] = {0, state_est_k1[2][2]*0.66, 0}
      l_leg_offset = vector.copy(lf)
      r_leg_offset = vector.copy(rf)
      trav_offset = {(rf[1] - COG[1]), (rf[2] - COG[2])}
      delta = vector.new{0,0,0,ahrs[7],ahrs[8],ahrs[9]}
      goal = vector.copy(trav_offset)
      COG_des = goal + vector.new{-0.08, 0.025}
      print('cog des')      
      util.ptable(COG_des)
      print('goal')
      util.ptable(goal)
    end
  elseif (state == 6) then --move COG
    local ratio = {1, 1}
    local tau = 2 - state_t --remaining time
    if state_t > 0.35 then 
      ratio[1] = (trav_offset[1]-(rf[1] - COG[1])) / (r_leg_offset[1] - rf[1])
      ratio[2] = (trav_offset[2]-(rf[2] - COG[2])) / (r_leg_offset[2] - rf[2])
      goal = {COG_des[1]/ratio[1], COG_des[2]/ratio[2]}
    end
    local x,xd,xdd = trajectory.minimum_jerk_step(hip_state[1], {goal[1],0,0}, tau, 0.004)
    local y,yd,ydd = trajectory.minimum_jerk_step(hip_state[2], {goal[2],0,0}, tau, 0.004)
    hip_state[1] = vector.new{x, xd, xdd} 
    hip_state[2] = vector.new{y, yd, ydd}
    delta[1] = hip_state[1][1]
    delta[2] = hip_state[2][1] 
    --print('ratio', ratio[1], ratio[2])
    storemove = {ratio[1], goal[1], (trav_offset[1]-(rf[1] - COG[1])), (r_leg_offset[1] - rf[1])}
    storemove2 = {ratio[2], goal[2], (trav_offset[2]-(rf[2] - COG[2])), (r_leg_offset[2] - rf[2])}
    --print('trav_offset')
    --util.ptable(trav_offset)
    --print('goal')
    --util.ptable(goal)
    --print('hip_state_x')
    --util.ptable(hip_state[1])
     --print('hip_state_y')
    --util.ptable(hip_state[2])
    qt = move_legs(delta)
    if state_t >= 2 then  -- true then 
      print("begin single support", t)
      util.ptable(delta)
      --print('qt')
      --util.ptable(qt)
      --util.ptable(joint_pos_sense)
      run = false
      state = 7
      state_t = 0
      joint_offset = vector.copy(qt) 
      qimp, qmid = find_stride_joint_angles(0.295, 0.25)
      delta = qmid - joint_offset 
      reset_states()
    end
  end
end

--------------------------------------------------------------------
--Initialize
--------------------------------------------------------------------
unix.usleep(5e5)
Platform.entry()
Platform.set_time_step(0.004)
print('timestep', Platform.get_time_step())
Proprioception.entry()
dcm:set_joint_enable(0,'all')
local set_values = dcm:get_joint_position_sensor('legs') 
dcm:set_joint_p_gain(1, 'all') -- position control
dcm:set_joint_p_gain(0, 'ankles')
dcm:set_joint_force({0, 0, 0, 0},'ankles')
dcm:set_joint_position(set_values)
dcm:set_joint_enable(1, 'all')
qt = vector.copy(set_values) 

local ident = "t1"
print('ident', ident)
local fw_log = assert(io.open("../Logs/fw_log"..ident..".txt","w"))
local fw_reg = assert(io.open("../Logs/fw_reg"..ident..".txt","w"))

function write_to_file(filename, data, test)
  for i = 1,#data do
    filename:write(data[i], ", ")
  end
  filename:write(t, "\n")
end

local store = vector.zeros(120000)
local ind = 1
local log_var_names = {}
function store_data(local_data)
  for i = 1, #local_data do
    for i2 = 1, #local_data[i] do
      --print(store_len, ind, 'are values')
      store[ind] = local_data[i][i2]
      ind = ind + 1
    end
  end
end

function store_data2(filename, local_data)
  for i = 1, #local_data do
    for i2 = 1, #local_data[i] do
      --print(store_len, ind, 'are values')
      filename:write(local_data[i][i2], ", ")
    end
  end
  filename:write(t, "\n")
end

function write_to_file2(filename, local_data, template)
  print('data length', #local_data)
  local local_ind = 1
  while local_ind < ind do
    for i = 1, #template do
      for i2 = 1, #template[i] do        
     	filename:write(local_data[local_ind], ", ")
        local_ind = local_ind + 1
      end
    end
    filename:write("\n")
  end
end

function write_reg(data)
  for i = 1, #data do
    fw_reg:write(#data[i], ", ")
  end
  fw_reg:write("\n")
  for i = 1, #data-1 do
    fw_reg:write(log_var_names[i], ", ")
  end
end

--------------------------------------------------------------------
--Main
--------------------------------------------------------------------
unix.usleep(1e6)
local t0 = unix.time()
t = t0
while run do 
  Platform.update()
  dt = Platform.get_time() - t 
  t = t + dt --simulation time
  Proprioception.update()
  state_t = state_t + dt --time used in state machine
  step = step + 1  --step number
--sensor updates
  update_joint_data()
  ahrs_update()
  update_force_torque()
  COP_update()
  foot_pose_update()
  compute_foot_state()
  COG_update() 
  state_machine(t)
  --if state >= 1 then orient_torso(0) end--modify qt's to upright torso 
  pid_torques = COG_controller(COG_des)
  --end
  joint_torques = update_joint_torques()
  update_observer()
  update_observer_double_support()
  joint_torques_sense = dcm:get_joint_force_sensor('legs')
--implement actions
  dcm:set_joint_position(qt, 'legs')  
  dcm:set_joint_force(joint_torques, 'legs')  
  --dcm:set_joint_force({0,0,0,0,0,0,0,0,0,0,0,0}, 'legs') 

  if (printdata)  then 
    data[1] = {joint_torques[5], joint_torques[6], joint_torques[11]}
    data[1][4] = joint_torques[12]
   -- data[2] = {ft_filt[3], ft_filt[5], ft_filt[6], ft_filt[9], ft_filt[11]}
   -- data[2][6] = ft_filt[12]
    data[2] = lf
    data[3] = grav_comp_torques
    data[4] = pid_torques
    data[5] = ahrs_filt
    data[6] = state_est_d1
    data[7] = {bias[1], foot_state[5], foot_state[11]}
    data[8] = ff_torques
    data[9] = COG
    data[10] = state_est_k1[1]
    data[11] = state_est_k1[2]
    --data[10] = lf
    --data[11] = COG
    --data[12] = joint_pos_sense
    --data[13] = ahrs_filt
    --data[14] = state_est_d1
    --data[15] =  bias
    --data[16] = {foot_state[5], foot_state[11]}
    --data[17] = storemove
    --data[18] = storemove2
    data[12] = {t} --robot only
    --
    log_var_names = {'jnt_torq', 'lf', 'grav_trq', 'pid_trq', 'bias'}
    log_var_names[6] = 'state_est_d1'
    log_var_names[7] = 'ratio_etc'
    log_var_names[8] = 'fftorques'
    log_var_names[9] = 'COG'
    log_var_names[10] = 'qt'
    log_var_names[11] = 'qt_test'
    log_var_names[12] = 't'
    --log_var_names[10] = 'lf'
    --log_var_names[11] = 'COG'
    --log_var_names[12] = 'joint_pos_sense'
    --log_var_names[13] = 'ahrs'
    --log_var_names[14] = 'state_est_d1'
    --log_var_names[15] = 'bias'
    --log_var_names[16] = 'foot_state'
    --log_var_names[17] = 'delta'
    --log_var_names[18] = 'delta2'
    store_data(data)  --robot only
    --store_data2(fw_log, data)  --webots only
  end

end
dcm:set_joint_force({0,0,0,0,0,0,0,0,0,0,0,0},'legs')
write_reg(data)
print('length data', #store)
write_to_file2(fw_log, store, data)  --robot only
print('steps: ', step)
Platform.exit()



