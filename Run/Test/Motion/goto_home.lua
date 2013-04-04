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
require('curses')
require('Config')
require('vector')
require('Kinematics')
require('filter')
require('Transform')
require('Proprioception')
require('Platform')
require('getch')
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
local ft_filt = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --filtered force torque data
local ft = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} -- raw force torques (debug only)
local lf = {}
local rf = {}
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
local l_cop, r_cop = {}, {}
local bias = {0.5, 0.5} -- proportion of weight on left and right feet
local test_foot_state = {[1] = 'l', [2] = '0'}
local hip_offset = 0
local state_est_k1 = {vector.new({-0.002, 0}), vector.new({-0.146, 0})} 
local state_act_k1 = {vector.new({0, 0}), vector.new({0, 0})}
local r_foot_pose_ref = {}
local l_foot_pose_ref = {}


------------------------------------------------------------------
--Control objects:
------------------------------------------------------------------
local COP_filters = {filter.new_second_order_low_pass(0.004, 20, 0.7), filter.new_second_order_low_pass(0.004, 20, 0.7)}

local torque_filters = {}
for i, index in pairs(joint.ankles) do
  torque_filters[index] = filter.new_low_pass(0.004, 20)--filter.new_second_order_low_pass(0.004, 40, 0.7) 
end

pgain, igain, dgain = 200, 120, 40
local COGx_pid = pid.new(0.004, pgain, igain, dgain)
COGx_pid:set_d_corner_frequency(20) 
local COGy_pid = pid.new(0.004, pgain, igain, dgain)
COGy_pid:set_d_corner_frequency(20)

pgain, igain, dgain = 200, 0, 20
local anklex_pid = pid.new(0.004, pgain, igain, dgain)
anklex_pid:set_d_corner_frequency(20) 
local ankley_pid = pid.new(0.004, pgain, igain, dgain)
ankley_pid:set_d_corner_frequency(20)

local COG_vel_filter = {}
COG_vel_filter[1] = filter.new_differentiator(0.004, 30, 0.5)
COG_vel_filter[2] = filter.new_differentiator(0.004, 30, 0.5)

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
  --bias[1] = 0.25*bias[1] + 0.75*ft_filt[3]/(ft_filt[3] + ft_filt[9]) --left
  --bias[2] = 0.25*bias[2] + 0.75*ft_filt[9]/(ft_filt[3] + ft_filt[9]) --right
  --for i = 1,2 do
    --bias[i] = math.min(bias[i], 1)
    --bias[i] = bias_filters[i]:update(bias[i])
  --end
  bias[1] = 0.5 --reverse comments when force torques are working ------------
  bias[2] = 0.5 --reverse comments when force torques are working ------------
end

function COG_controller_act(des_loc) --uses loc wrt foot
  --controls the COG with the torque on the ground to maintain a 
  --COG position given by "des_loc", 
  local torques = vector.new{0, 0}
  local foot_pos = {0, 0}
  local feet_on_gnd = foot_state[5] + foot_state[11]
  if test_foot_state[1] == 'l' then --use left foot as reference
    foot_pos = vector.copy(lf)
  else  -- use right foot as reference
    foot_pos = vector.copy(rf)
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

function COG_controller()
--compute x and y torques based on ankle pitch and roll
  local torques = vector.new{0, 0}
  ankley_pid:set_setpoint(qt[5])
  torques[1] = 0
  torques[2] = ankley_pid:update(raw_pos[5])
  return torques
end

local COG_rate = 0
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
    if i == 2 then
      u[i] = math.max(math.min(u[i],20),-20)
    else
      u[i] = -1*math.max(math.min(u[i],10),-10)
    end
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
  --if feet_on_gnd == 1 then  --remove comments when force torques work ----------
   -- tor_comp = vector.copy(torques)
  --elseif feet_on_gnd == 2 then
    axis = vector.new{lf[1]-rf[1], lf[2]-rf[2]}
    local mag = math.sqrt(axis[1]*axis[1] + axis[2]*axis[2])
    axis = axis/mag
    dot = axis[1]*torques[1] + axis[2]*torques[2]
    tor_comp[1] = dot*axis[1]
    tor_comp[2] = dot*axis[2]
  --end
  return tor_comp
end

local grav_off = false
function update_joint_torques()
  --computes gravity torque, applies bias, applies wrt ankles
  --regulates to max and min, filters, and returns result
  local torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
  calculate_bias() 
  --grav_comp_torques = gravity_comp() --remove comments when force torques work --------
  local t_x_y =  grav_comp_torques + pid_torques + ff_torques --sums torques
  t_x_y = torque_components_wrt_feet_on_gnd(t_x_y)
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
    --if math.abs(ft_filt[v])>30 then f = 1 else f = 0 end --remove comments when force torques work --------
    if true then f = 1 end  --remove when force torques work ---------------
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

function ahrs_update() --move
  ahrs = dcm:get_ahrs()
  for i = 1, 9 do
    ahrs_filt[i] = ahrs_filters[i]:update(ahrs[i])
  end 
end

function COG_update()
  local COG_temp = pcm:get_cog()
  for i = 1, 3 do
    COG[i] = COG_filters[i]:update(COG_temp[i])
  end
end

local tp = {-0.3, 0.3}
local tr = {-0.3, 0.3}
trial = 1

local poses = {{0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
               {0.08, 0.00, 0.00, 0.00, 0.00, 0.00},
               {-0.06, 0.00, 0.00, 0.00, 0.00, 0.00},
               {0.00, 0.10, 0.00, 0.00, 0.00, 0.00},
               {0.08, 0.10, 0.00, 0.00, 0.00, 0.00},
               {-0.06, 0.10, 0.00, 0.00, 0.00, 0.00},
               {0.00, 0.00, 0.00, 0.2, 0.00, 0.00},
               {0.00, 0.00, 0.00, 0.00, 0.2, 0.00},
               {0.00, 0.00, 0.00, -0.2, 0.00, 0.00},
               {0.00, 0.00, 0.00, 0.00, -0.2, 0.00}}

function generate_pose_angles(torso)
  local torso_pose = Transform.pose(torso)
  q_desired = Kinematics.inverse_pos_legs(l_foot_pose_ref, r_foot_pose_ref, torso_pose)
  return q_desired
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

function sagital_balance(offset) --controls pitch axes
  local gains = vector.new{337.7, -58.8, 13.5, -2.7}
  local states = vector.new{joint_pos[5]-offset[1], joint_pos[3]-offset[2], joint_vel[5], joint_vel[3]}
  local acc = gains*states
  return acc
end

------------------------------------------------------------------------
--State machine
------------------------------------------------------------------------
function state_machine(t) 
  if (state == 0) then
    print("goto home")
    --go to original position
    if state_t >=0.5 then
      joint_offset = dcm:get_joint_position_sensor('legs') 
      q_goal = vector.zeros(12)
      delta = q_goal - joint_offset 
      --util.ptable(delta)
      print('state = 5')
      state = 1
      state_t = 0
    end
  elseif (state == 1) then
    --move to final state
    local percent = trajectory_percentage(1, 3, state_t)
    qt = percent*delta + joint_offset
    --print('r_ankle pitch', qt[11])
    if (percent >= 1) then  
      print('complete')
      run = false
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
util.ptable(set_values) 
dcm:set_joint_p_gain(1, 'all') -- position control
--dcm:set_joint_position_p_gain(0, 'ankles')
--dcm:set_joint_damping(0, 'ankles')
--dcm:set_joint_force({0, 0, 0, 0},'ankles')
dcm:set_joint_position(set_values)
dcm:set_joint_enable(1, 'all')
qt = vector.copy(set_values) 
printdata = false

--------------------------------------------------------------------
--Main
--------------------------------------------------------------------
--local t0 = unix.time()
--t = t0
print('begin')
while run do 
  Platform.update()
  dt = Platform.get_time() - t 
  t = t + dt --simulation time
  state_t = state_t + dt --time used in state machine
  step = step + 1  --step number
  Proprioception.update()
--sensor updates
  update_joint_data()
  ahrs_update()
  update_force_torque()
  COP_update()
  foot_pose_update()
  compute_foot_state()
  COG_update() 
  state_machine(t)
  joint_torques = update_joint_torques()
  update_observer()
  joint_torques_sense = dcm:get_joint_force_sensor('legs')
--implement actions
  dcm:set_joint_position(qt, 'legs')  
end
print('time done')
Platform.exit()



