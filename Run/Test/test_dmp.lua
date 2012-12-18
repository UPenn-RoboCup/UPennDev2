dofile('../include.lua')

require('dmp')
require('unix')
require('numlua')
require('gnuplot')
require('trajectory')

local x0     = {0} 
local g0     = {1}
local tau    = 1
local dt     = 0.001
local nbasis = 10 

local t0, t1
local primitive = dmp.new(1)
local minimum_jerk = trajectory.minimum_jerk(x0, g0, tau)

primitive:learn_minimum_jerk_trajectory(x0, g0, tau, nbasis)
primitive:set_time_step(dt)

-- integrate dmp
local x_learned = {}
local x_target = {}
local t = {}
local f = {}
local s = {}

local nonlinearity = primitive:get_transform_system(1):get_nonlinearity()


primitive:reset()

t0 = unix.time()
for i = 1,math.floor(tau/dt) do

  -- update time
  t[i] = (i - 1)*dt

  -- update goal position
  if(t[i] > 0.5) then primitive:set_goal_position({0.5}) end

  -- update parameters
  x_target[i] = minimum_jerk(t[i])
  x_learned[i] = primitive:get_position(1)
  s[i] = primitive:get_phase()
  f[i] = nonlinearity:predict(s[i])

  -- integrate dmp
   primitive:integrate()
end

t1 = unix.time()

print('integration time', t1 - t0)

gnuplot.figure()
gnuplot.plot(
  {'x_learned(t)', t, x_learned, '-'},
  {'x_target(t)', t, x_target, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'f(t)', t, f, '-'},
  {'s(t)', t, s, '-'}
)
