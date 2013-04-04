--------------------------------------------------------
--One Legged Swing controller
--------------------------------------------------------
dofile('../../include.lua')

require('dcm')
require('pid')
require('unix')
require('util')
require('Platform')
require('walk')
require('curses')
require('Config')
require('vector')
require('Kinematics')
require('matrix')
require('filter')
require('Transform')

if not string.match(Config.platform.name, 'webots') then
  return
end
local joint = Config.joint
--------------------------------------------------------------------
-- Parameters
--------------------------------------------------------------------
--local stats = util.loop_stats(100)
local t, dt = Platform.get_time(), 0
local COG_ratio = 1.182 --m_total/(m_torso+2*leg_length_ratio*m_leg)-- tune this and just set it instead
local qt = {} --desired joint angles 
local joint_pos = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --current joint positions
local joint_vel = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --current joint velocity
local joint_pos_sense = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local raw_pos = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local joint_vel_raw = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
local joint_torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --desired joint torques
local foot_state = {[5] = 1, [6] = 1, [11] = 1, [12] = 1} --left foot, right foot on ground? 1 = yes, 0 = no
local ahrs_filt = {}
local COP_filt = {0, 0} --filtered COP location wrt torso projection
local COG = {0, 0}
local ft_filt = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} --filtered force torque data
local ft = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local l_leg_offset = vector.new{0, 0, 0} --xyz pos at beginning of move
local r_leg_offset = vector.new{0, 0, 0}
local joint_offset = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local state_t = 0 --keps record of time in each state
local state = 0 --keeps record of which state program is in
local run = true --boolean to quit program
local move_leg = vector.new{0, 0, 0} 
local torso_position = vector.new{0, 0, 0}
local step = 1
local left_cop = {} --these are duplicates of l_cop_t
local right_cop = {}
local force_torque = {}
local printdata = false
local temp_accel = 0
local joint_torques_temp = 0
local l_cop_t = {}
local r_cop_t = {}
local lf, rf = {}, {}
local grav_comp_torques = {0, 0}
local bias = {['L'] = 0.5, ['R'] = 0.5}

------------------------------------------------------------------
--Control objects:
------------------------------------------------------------------

local filter_b = {0.013251, 0.026502, 0.013251} --10 break freq, ts=0.004, chsi=0.7
local filter_a = {1, -1.6517, 0.70475}
local COP_filters = {filter.new(filter_b, filter_a), filter.new(filter_b, filter_a)}

local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 1 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
local pgain, igain, dgain = 300, 0, 60  --need different gains in different joints
local position_pids = {}
local torque_filters = {}
for i, index in pairs(joint.ankles) do
  position_pids[index] = pid.new(0.004, pgain, igain, dgain)
  position_pids[index]:set_d_filter(filter_b, filter_a)
  torque_filters[index] = filter.new_second_order_low_pass(0.004, 40, 0.7) 
end

local pos_filters = {}
for i, index in pairs(joint.all) do
  pos_filters[index] = filter.new_second_order_low_pass(0.004, 20, 0.5)
end

local vel_filters = {}
local vel_filters_raw = {}
local filter_b = {   0.1094,    0.1094,   -0.1094,   -0.1094} --5 freq, 3 order butter, 1 dt
local filter_a = {  1.0000,   -2.7492,    2.5288,   -0.7779}
for i, index in pairs(joint.all) do
  vel_filters[index] = filter.new(filter_b, filter_a)
  vel_filters_raw[index] =  filter.new_differentiator(0.004, 5)
end

local ft_filters = {}
for i = 1, 12 do
  ft_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end

local ahrs_filters = {}
for i = 1, 6 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 20, 0.7)
end
for i = 7, 9 do
  ahrs_filters[i] = filter.new_second_order_low_pass(0.004, 40, 0.7)
end
------------------------------------------------------------------
--Utilities:
------------------------------------------------------------------
function cop(ft)  -- finds the x and y position of the COP under the foot
  local cop_left, cop_right = {}, {}
  if (ft[3]~=0) then
    cop_left[1] = -ft[5]/ft[3]
    cop_left[2] = ft[4]/ft[3]
    foot_state[5], foot_state[6] = 1, 1
  else
    cop_left[1] = 999
    cop_left[2] = 999
    foot_state[5], foot_state[6] = 0, 0
  end
  if (ft[9]~=0) then
    cop_right[1] = -ft[11]/ft[9] 
    cop_right[2] = ft[10]/ft[9]
    foot_state[11], foot_state[12] = 1, 1
  else
    cop_right[1] = 999
    cop_right[2] = 999
    foot_state[11], foot_state[12] = 0, 0
  end
  --print('left, right', cop_left[1], cop_right[1])
  l_cop_t = cop_left
  r_cop_t = cop_right
  return cop_left, cop_right
end

function robot_cop(ft, cop_left, cop_right)
  --returns the center of pressure for the whole robot
  local cop_robot = {}
  local l_foot_pos, r_foot_pos = foot_rel_torso_proj()
  lf, rf = l_foot_pos, r_foot_pos
  if not(ft[3] == 0 and ft[9] == 0) then
    cop_robot[1] = ((cop_left[1] + l_foot_pos[1])*ft[3] + (cop_right[1] + r_foot_pos[1])*ft[9])/(ft[3]+ft[9])
    cop_robot[2] = ((cop_left[2] + l_foot_pos[2])*ft[3] + (cop_right[2] + r_foot_pos[2])*ft[9])/(ft[3]+ft[9])
  else
     cop_robot = {0, 0}
  end
  return cop_robot
end

function COP_update()
  force_torque = dcm:get_force_torque()
  ft = force_torque --log force torque data
  left_cop, right_cop = cop(force_torque) --compute COP
  COP = robot_cop(force_torque, left_cop, right_cop) 
  COP_filt[1] = COP_filters[1]:update(COP[1])
  COP_filt[2] = COP_filters[2]:update(COP[2])
end

function foot_rel_torso_proj()
  --computes foot position relative to the torso coord frame projection on ground
  local torso_rpy = {0, 0, 0, ahrs_filt[7], ahrs_filt[8], ahrs_filt[9]}
  local lf, rf = Kinematics.forward_legs(joint_pos, torso_rpy)
  local left_foot_pos = vector.new{lf[1][4], lf[2][4], lf[3][4]}
  local right_foot_pos = vector.new{rf[1][4], rf[2][4], rf[3][4]}
  return left_foot_pos, right_foot_pos
end

function trajectory_percentage(final_pos,time_final,time)
  local velocity = 5/4*final_pos/time_final
  local accel = velocity/(time_final/5)
  
  local percent = 0
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
  return percent
end

function calculate_bias()
  local bias, tempx, tempy = 0, 0, 0
  tempx = (COG[1]-lf[1])/(rf[1]-lf[1])
  tempy = (COG[2]-lf[2])/(rf[2]-lf[2])
  bias = (tempx + tempy)/2
  return bias
end

function update_joint_torques(foot_state_l)
  -- update force commands
  local torques = vector.new{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} 
  local temp = 0
  for i, pid_loop in pairs(position_pids) do
    pid_loop:set_setpoint(qt[i])
    torques[i] = pid_loop:update(joint_pos_sense[i])  --pid
  end
  bias.L = 0.5*bias.L + 0.5*ft_filt[3]/(ft_filt[3] + ft_filt[9])
  bias.R = 0.5*bias.R + 0.5*ft_filt[9]/(ft_filt[3] + ft_filt[9])
  --print('bias', bias.L, bias.R)
  bias.L = math.min(bias.L, 1)
  bias.R = math.min(bias.R, 1)
  torques[5] = torques[5] --+  grav_comp_torques[1]*bias.L
  torques[11] = torques[11] --+ grav_comp_torques[1]*bias.R
  torques = regulate_ankle_torques(torques) --max min
  for i, torque_loop in pairs(torque_filters) do
    torques[i] = torque_filters[i]:update(torques[i])
  end
  return torques
end

function regulate_ankle_torques(torque)
  local max = {[5] = 11, [6] = 5, [11] = 11, [12] = 5 }
  local min = {[5] = -11, [6] = -5, [11] = -11, [12] = -5 }
  local f = 0
  local tbias = calculate_bias()
  for i,v in pairs{[5] = 3, [6] = 3, [11] = 9, [12] = 9} do
    if math.abs(ft_filt[v])>50 then f = 1 else f = 0 end
    torque[i] = math.min(torque[i], f*max[i])
    torque[i] = math.max(torque[i], f*min[i])
  end
   print('t torq', t, torque[6], tbias)
  return torque
end

function update_joint_data()
  raw_pos = dcm:get_joint_position_sensor()
  joint_pos = dcm:get_joint_position()
  for i, filter_loop in pairs(pos_filters) do
    joint_pos_sense[i] = filter_loop:update(raw_pos[i])--position filters
  end
  for i, filter_loop in pairs(vel_filters) do
    joint_vel[i] = filter_loop:update(raw_pos[i])--velocity filters
  end    
  for i, filter_loop in pairs(vel_filters_raw) do
    joint_vel_raw[i] = filter_loop:update(raw_pos[i])--velocity filters
  end 
end

function update_force_torque()
--filters the force torque data for use
  local ft = dcm:get_force_torque() --debuggin
  for i = 1, 12 do
    ft_filt[i] = ft_filters[i]:update(ft[i])
  end 
end

function COG_update(pos)
  --returns location of COG wrt base frame
  local jnt_vec_x = vector.new{0, 0, 0, 1}
  local jnt_vec_y = vector.new{0, 0, 0, 1}
  local bsx = vector.new{0.0732, -0.0742, -0.0295, 0.0046}
  local bsy = vector.new{-0.0737, 0.1025, 0.0201, 0.0004}
  local COG_l = {}
  --local pos = joint_pos_sense
  --need to check on ahrs orientation
  jnt_vec_x[1] = math.sin(ahrs_filt[8])
  jnt_vec_x[2] = math.sin(ahrs_filt[8] + pos[3]) + math.sin(ahrs_filt[8] + pos[9]) 
  jnt_vec_x[3] = math.sin(ahrs_filt[8] + pos[3] + pos[4]) 
  jnt_vec_x[3] = jnt_vec_x[3] +  math.sin(ahrs_filt[8] + pos[9] + pos[10]) 

  jnt_vec_y[1] = math.sin(ahrs_filt[7])
  jnt_vec_y[2] = math.sin(ahrs_filt[7] + pos[2]) + math.sin(ahrs_filt[7] + pos[8])
  jnt_vec_y[3] = math.sin(ahrs_filt[7] + pos[2])*pos[3]*pos[4]
  jnt_vec_y[3] = jnt_vec_y[3] + math.sin(ahrs_filt[7] + pos[8])*pos[9]*pos[10]

  COG_l[1] = vector.mul(jnt_vec_x, bsx)
  COG_l[2] = vector.mul(jnt_vec_y, bsy)
  return COG_l
end

function ahrs_update()
  local ahrs = dcm:get_ahrs()
  for i = 1, 9 do
    ahrs_filt[i] = ahrs_filters[i]:update(ahrs[i])
  end 
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
  local l_foot_pos = {x_offset-torso[1], y_offset-torso[2], z_offset-torso[3], 0, 0, 0} 
  local r_foot_pos = {x_offset-torso[1],-y_offset-torso[2], z_offset-torso[3], 0, 0, 0} 
  local torso_pos = {0, 0, 0, 0, 0, 0}

  local qStance = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)
  return qStance  
end

function move_r_leg(r_leg)
  local temp = r_leg_offset + r_leg
  local l_foot_pos = {l_leg_offset[1], l_leg_offset[2], l_leg_offset[3], 0, 0, 0} 
  local r_foot_pos = {temp[1], temp[2], temp[3], 0, 0, 0}  --use vector here to improve syntax
  local torso_pos = {0, 0, 0, 0, 0, 0} 

  local qLeg = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)
  return qLeg
end

function desired_cop_location(COG_shift_x, COG_shift_y)
  local torso_shift = vector.new{0, 0, 0, 0, 0, 0}
  torso_shift[1] = COG_shift_x*COG_ratio
  torso_shift[2] = COG_shift_y*1.30--COG_ratio
  return torso_shift
end

function sagital_balance(offset) --controls pitch axes
  local gains = vector.new{337.7, -58.8, 13.5, -2.7}
  local states = vector.new{joint_pos[5]-offset[1], joint_pos[3]-offset[2], joint_vel[5], joint_vel[3]}
  local acc = vector.mul(gains, states)
  --print('s1 2 3 4', joint_pos[5]-offset[1], joint_pos[3]-offset[2], joint_vel[5], joint_vel[3], acc)
  return acc
end

function swing_balance()
  local C_alpha = 1.352
  local p_fx = 10
  local temp_torque = 0
  temp_torque = -1*C_alpha*joint_acc[9] + ft_filt[1]*p_fx
  joint_torques_temp = temp_torque
end

function gravity_comp(des_cog)
  --works for left foot only at this point
  local mg = 230
  local kp = 0
  --local torso_rpy = {0, 0, 0, ahrs_filt[7], ahrs_filt[8], ahrs_filt[9]}
  --local lf, rf = Kinematics.forward_legs(joint_pos, torso_rpy)
  local ex = COG[1] - lf[1] --lever arm of COG wrt left foot in x
  local ey = COG[2] - lf[2] -- in y 
  local tx = ex*mg + (des_cog[1] - ex)*kp
  local ty = ey*mg + (des_cog[2] - ey)*kp
  return {tx, ty}
end

------------------------------------------------------------------------
--State machine
------------------------------------------------------------------------
function state_machine(t) 
  if (state == 0) then
    local left_leg_position = Kinematics.forward_l_leg(joint_pos)
    l_leg_offset = {left_leg_position[1][4], left_leg_position[2][4], left_leg_position[3][4]}
    torso = vector.new{0, 0, -0.05, 0, 0, 0}  --{0,0,0,0,0.2,0}
    if state_t >= 0.5 then
      print("move to ready", t)
      state = 1
      state_t = 0
    end
  elseif (state == 1) then 
    --move to ready position
    local percent = trajectory_percentage(1, 2, state_t)
    qt = move_legs(percent*torso)
    if (percent >= 1) then  --when movement is complete, advance state
      state = 2
      state_t = 0
      print("wait for 0.5", t)
    end
  elseif (state == 2) then
    --wait specified time
    if (state_t > 0.5) then  
      --print('state = 3')
      print(t)
      state = 3
      state_t = 0
    end
  elseif (state == 3) then 
    --measure COG
    local left_foot_xy, right_foot_xy = foot_rel_torso_proj()  --foot locations
    COG_shift = {left_foot_xy[1] - COP_filt[1], left_foot_xy[2] - COP_filt[2], 0}
    torso = desired_cop_location(COG_shift[1], COG_shift[2])
    local left_leg_position = Kinematics.forward_l_leg(joint_pos)  
    l_leg_offset = {left_leg_position[1][4], left_leg_position[2][4], left_leg_position[3][4]}
    state = 4
    state_t = 0
    print("move over COG", t)
  elseif (state == 4) then
    --move to over COG
    local percent = trajectory_percentage(1, 2, state_t)
    local left_foot_xy, right_foot_xy = foot_rel_torso_proj()
    qt = move_legs(percent*torso)
    if (percent >= 1) then  --when movement is complete, advance state
      state = 5
      state_t = 0
      run = false
      print("wait 0.7", t)
    end
  elseif (state == 5) then
    local left_foot_xy, right_foot_xy = foot_rel_torso_proj()
    if (state_t > 0.7) then
      left_foot_xy, right_foot_xy = foot_rel_torso_proj()
      state = 6
      state_t = 0
    end
  elseif (state == 6) then
    -- prepare to lift left foot
    local left_leg_position = Kinematics.forward_l_leg(joint_pos)  
    l_leg_offset = {left_leg_position[1][4], left_leg_position[2][4], left_leg_position[3][4]}
    local right_leg_position = Kinematics.forward_r_leg(joint_pos)  
    r_leg_offset = {right_leg_position[1][4], right_leg_position[2][4], right_leg_position[3][4]}
    move_leg =vector.new{0, 0.02, 0.05}
    state = 7
    state_t = 0
    print("lift leg", t)
  elseif (state == 7) then
    --lift left leg
    local percent = trajectory_percentage(1, 1, state_t)
    qt = move_r_leg(percent*move_leg)
    if (percent >= 1) then
      state = 9
      state_t = 0
      print("wait 2", t)
    end
  elseif (state == 8) then
    local percent = trajectory_percentage(1, 1, state_t)
    qt[9] = joint_offset[9] + percent*-0.2
    swing_balance()
    if percent >= 1 then
      state = 10
      state_t = 0
    end
  elseif (state == 9) then
    if state_t>=2 then
      run = false
    end
  end
end

--------------------------------------------------------------------
--Initialize
--------------------------------------------------------------------
Platform.entry()
dcm:set_joint_enable(0,'all')
local set_values = dcm:get_joint_position('all') --records original joint pos
dcm:set_joint_stiffness(1, 'all') -- position control
dcm:set_joint_stiffness(0, 'ankles')
dcm:set_joint_force({0, 0, 0, 0},'ankles')
dcm:set_joint_enable(1, 'all')
qt = set_values
printdata = true

local fw_joint_pos = assert(io.open("Logs/fw_joint_pos.txt","w"))
local fw_grav_comp_torques = assert(io.open("Logs/fw_grav_comp_torques.txt","w"))
local fw_qt = assert(io.open("Logs/fw_qt.txt","w"))
local fw_raw_pos = assert(io.open("Logs/fw_raw_pos.txt","w"))
local fw_joint_pos_sense = assert(io.open("Logs/fw_joint_pos_sense.txt","w"))
local fw_joint_vel_raw = assert(io.open("Logs/fw_joint_vel_raw.txt","w"))
local fw_joint_vel = assert(io.open("Logs/fw_joint_vel.txt","w"))
local fw_joint_torques = assert(io.open("Logs/fw_joint_torques.txt","w"))
local fw_joint_force = assert(io.open("Logs/fw_joint_force.txt","w"))
local fw_joint_force_sense = assert(io.open("Logs/fw_joint_force_sense.txt","w"))
local fw_COG = assert(io.open("Logs/fw_COG.txt","w"))
local fw_ft_filt = assert(io.open("Logs/fw_ft_filt.txt","w"))
local fw_ft = assert(io.open("Logs/fw_ft.txt","w"))
local fw_lr_cop_t = assert(io.open("Logs/fw_lr_cop_t.txt","w"))
local fw_COP_filt = assert(io.open("Logs/fw_COP_filt.txt","w"))
local fw_ahrs_filt = assert(io.open("Logs/fw_ahrs_filt.txt","w"))
local fw_COG_des = assert(io.open("Logs/fw_COG_des.txt","w"))
local fw_lf = assert(io.open("Logs/fw_lf.txt","w"))

function write_to_file(filename, data, test)
  for i = 1,#data do
    filename:write(data[i], ", ")
  end
  filename:write(t, "\n")
end

--------------------------------------------------------------------
--Main
--------------------------------------------------------------------
while run do   --run step<20
  Platform.update()
  dt = Platform.get_time() - t --
  t = t + dt --simulation time
  state_t = state_t + dt --time used in state machine
  step = step + 1  --step number

  update_joint_data()
  ahrs_update()
  update_force_torque()
  COP_update()
  COG = COG_update(joint_pos_sense)
  state_machine(t) 
  dcm:set_joint_position(qt,'all')
  grav_comp_torques = gravity_comp({0,0}) 
  joint_torques = update_joint_torques(foot_state)
  COG_des = COG_update(joint_pos)
  dcm:set_joint_force(joint_torques, 'all')  

  if (printdata) then -- mod_print == 0 and
    write_to_file(fw_grav_comp_torques, grav_comp_torques)
    write_to_file(fw_COG_des, COG_des)
    write_to_file(fw_joint_pos, joint_pos)
    write_to_file(fw_joint_pos_sense, joint_pos_sense)
    write_to_file(fw_raw_pos, raw_pos)
    write_to_file(fw_joint_vel_raw, joint_vel_raw)
    write_to_file(fw_joint_vel, joint_vel)
    write_to_file(fw_qt, qt)
    write_to_file(fw_joint_torques, joint_torques)
    write_to_file(fw_joint_force, dcm:get_joint_force())
    write_to_file(fw_joint_force_sense, dcm:get_joint_force_sensor())
    write_to_file(fw_COG, COG)
    write_to_file(fw_ft_filt, ft_filt)
    write_to_file(fw_ft, ft)
    write_to_file(fw_lr_cop_t, {l_cop_t[1], l_cop_t[2], r_cop_t[1], r_cop_t[2]})
    write_to_file(fw_COP_filt, COP_filt)
    write_to_file(fw_ahrs_filt, ahrs_filt)
    write_to_file(fw_lf, lf)
  end
end
Platform.exit()


