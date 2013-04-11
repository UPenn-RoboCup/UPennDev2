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
for i, index in pairs(joint.ankles) do
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
for i, index in pairs(joint.legs) do
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

------------------------------------------------------------------------
--State machine
------------------------------------------------------------------------
function state_machine(t) 
  if (state == 0) then
    if state_t >= 1 then
      print('ident_stride', ident)
      print("move to ready", t)
      state = 1
      state_t = 0
      run = false
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
  --compute_foot_state()
  --local joint_pos_COG = vector.copy(joint_pos) --
  --joint_pos_COG[5] = joint_pos_sense[5]
  --joint_pos_COG[6] = joint_pos_sense[6]
  --joint_pos_COG[11] = joint_pos_sense[11]
  --joint_pos_COG[12] = joint_pos_sense[12]
  COG = pcm:get_cog()--COG_update(joint_pos_COG)
  state_machine(t)
  --if state >= 1 then orient_torso(0) end--modify qt's to upright torso
  --if walk == true then
    --pid_torques = COG_walk()
  --else 
    --pid_torques = COG_controller({0.0, 0})
  --end
  --if pid_override then  pid_torques[2] = 0 end --, pid_torques[1] = 0,
  --joint_torques = update_joint_torques()
  --update_observer()
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
    --write_to_file(fw_pid_terms, {COGx_pid.p_term, COGx_pid.i_term, COGx_pid.d_term, COG_rate, t})
  end
end
Platform.exit()



