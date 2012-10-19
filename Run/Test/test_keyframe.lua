dofile('../include.lua')

require('dcm')
require('Body')
require('unix')
require('getch')
require('Config')
require('gnuplot')
require('keyframe')

local motion = tonumber(arg[1]) or arg[1]
local q_joints, t_joints = {}, {}
local keyframe_table = require(Config.platform.keyframe_table)

-- Setup 
-----------------------------------------------------

dcm:set_joint_stiffness(1, 'all')
dcm:set_joint_damping(0, 'all')
dcm:set_joint_force(0, 'all')
dcm:set_joint_position(dcm:get_joint_position_sensor())
dcm:set_joint_velocity(0, 'all')
dcm:set_joint_enable(1, 'all')

keyframe:load_keyframe_table(keyframe_table)

-- Play keyframe motion
-----------------------------------------------------

Body.entry()
keyframe:entry()

if keyframe:play(motion) then
  print('playing... press any key to stop')
else
  print('usage: lua test_keyframe.lua [motion]')
  os.exit()
end

local t0 = Body.get_time() 
while (keyframe:update() ~= 'done') and (not getch.nonblock()) do
  Body.update()
  t_joints[#t_joints+1] = Body.get_time() - t0
  q_joints[#q_joints+1] = dcm:get_joint_position()
  unix.usleep(5000)
end

keyframe:exit()
Body.exit()

-- Plot the joint trajectories
-----------------------------------------------------

local plotvars = {}
for i = 1,joint.count do
  local q = {}
  for k = 1,#q_joints do
    q[k] = q_joints[k][i]
  end
  plotvars[i] = {joint.id[i], t_joints, q, '-'}
end

gnuplot.figure()
gnuplot.plot(unpack(plotvars))
