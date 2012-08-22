dofile('../include.lua')

require('acm')
require('scm')
require('Body')
require('unix')
require('getch')
require('Config')
require('gnuplot')
require('keyframe')

local action = tonumber(arg[1]) or arg[1]
local qJoints, tJoints = {}, {}
local action_table = require(Config.platform.action_table)

-- Setup 
-----------------------------------------------------

acm:set_joint_position(scm:get_joint_position())
acm:set_joint_mode(0, 'all')
acm:set_joint_enable(1, 'all')

keyframe:load_action_table(action_table)

-- Play keyframe action
-----------------------------------------------------

Body.entry()
keyframe:entry()

if keyframe:play(action) then
  print('playing... press any key to stop')
else
  print('usage: lua test_keyframe.lua [action]')
  os.exit()
end

local t0 = Body.get_time() 
while (keyframe:update() ~= 'done') and (not getch.nonblock()) do
  Body.update()
  tJoints[#tJoints+1] = Body.get_time() - t0
  qJoints[#qJoints+1] = acm:get_joint_position()
  unix.usleep(5000)
end

keyframe:exit()
Body.exit()

-- Plot the joint trajectories
-----------------------------------------------------

local plotvars = {}
for i = 1,joint.count do
  local q = {}
  for k = 1,#qJoints do
    q[k] = qJoints[k][i]
  end
  plotvars[i] = {joint.id[i], tJoints, q, '-'}
end

gnuplot.figure()
gnuplot.plot(unpack(plotvars))
