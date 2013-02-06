dofile('../include.lua')

require('dmp')
require('unix')
require('gnuplot')

local n_basis = 10  -- number of basis functions
local dt = 0.001   -- integrator time step
local tau = 1      -- movement duration
local xdata = {}   -- training trajectory
local tdata = {}   -- training sample times

-- initialize example trajectory
for i = 1, 10000 do
  tdata[i] = (i - 1)/(10000 - 1)
  xdata[i] = math.sin(math.pi*tdata[i]/2)
end

-- initialize dmp
local primitive = dmp.new(1, n_basis)
primitive:set_time_step(dt)
local fdata, sdata = primitive:learn_trajectory({xdata}, tdata)
primitive:reset()

local flearn = {}
local slearn = {}
local xlearn = {}
local tlearn = {}
local nonlinearity = primitive:get_transform_system(1):get_nonlinearity()

-- integrate dmp
t0 = unix.time()
for i = 1, math.floor(tau/dt) do
  primitive:integrate()
  tlearn[i] = i*dt
  xlearn[i] = primitive:get_position(1)
  slearn[i] = primitive:get_phase()
  flearn[i] = nonlinearity:predict(slearn[i])
end
print('integration time', unix.time() - t0)

gnuplot.figure()
gnuplot.plot(
  {'x_learned(t)', tlearn, xlearn, '-'},
  {'x_target(t)', tdata, xdata, '-'}
)

gnuplot.figure()
gnuplot.plot(
  {'f_learned(s)', slearn, flearn, '-'},
  {'f_target(s)', sdata, fdata[1], '-'}
)
