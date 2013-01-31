dofile('../../include.lua')

require('Body')
require('Proprioception')
require('trajectory')
require('Transform')
require('stepRMP')
require('gnuplot')
require('dcm')

local LOOP = true
local SLOW_MO = false
local INIT_STANCE = false
local PARAMETER_FILE = '../../../Data/parameters_stepRMP_WebotsASH_train0.lua'

local period = 0.7
local n_samples = 5000

-- define periodic torso trajectories
--------------------------------------------------------------------------------
--[[
local velocity = {0.25, 0, 0}

local start_state = {
  { 0.00, 0.07, 0.00}, -- x pos, vel, acc
  { 0.00, 0.20, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local via_state = {
  { 0.00,-0.16, 0.00}, -- x pos, vel, acc
  { 0.05, 0.00, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}
--]]

--[[
local velocity = {0.1875, 0, 0}

local start_state = {
  { 0.00, 0.05, 0.00}, -- x pos, vel, acc
  { 0.00, 0.20, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local via_state = {
  { 0.00,-0.15, 0.00}, -- x pos, vel, acc
  { 0.05, 0.00, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}
--]]

--[[
local velocity = {0.125, 0, 0}

local start_state = {
  { 0.00, 0.02, 0.00}, -- x pos, vel, acc
  { 0.00, 0.20, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local via_state = {
  { 0.00,-0.08, 0.00}, -- x pos, vel, acc
  { 0.05, 0.00, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}
--]]

--[[
local velocity = {0.0625, 0, 0}

local start_state = {
  { 0.00, 0.01, 0.00}, -- x pos, vel, acc
  { 0.00, 0.20, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local via_state = {
  { 0.00,-0.04, 0.00}, -- x pos, vel, acc
  { 0.05, 0.00, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}
--]]

local velocity = {0, 0, 0}

local start_state = {
  { 0.00, 0.00, 0.00}, -- x pos, vel, acc
  { 0.00, 0.20, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local via_state = {
  { 0.00, 0.00, 0.00}, -- x pos, vel, acc
  { 0.05, 0.00, 0.00}, -- y pos, vel, acc
  { 0.00, 0.00, 0.00}, -- z pos, vel, acc
}

local goal_state = {{}, {}, {}}
for i = 1, 3 do
  goal_state[1][i] = start_state[1][i] -- x (periodic)
  goal_state[2][i] =-start_state[2][i] -- y (antiperiodic)
  goal_state[3][i] = start_state[3][i] -- z (periodic)
end

local lift_trajectory = {{}, {}, {}}
local land_trajectory = {{}, {}, {}}
for i = 1, 3 do
  lift_trajectory[i] = trajectory.minimum_jerk(
    start_state[i],
    via_state[i],
    period/2
  )
  land_trajectory[i] = trajectory.minimum_jerk(
    via_state[i],
    goal_state[i],
    period/2
  )
end

local xdata = {{}, {}, {}}
local tdata = {}
for i = 1, n_samples do
  tdata[i] = (i-1)*period/n_samples
  for j = 1, 3 do
    if (tdata[i] < period/2) then
      xdata[j][i] = lift_trajectory[j](tdata[i])
    else
      xdata[j][i] = land_trajectory[j](tdata[i] - period/2)
    end
  end
end

-- initialize step controller and train torso rmp
--------------------------------------------------------------------------------
Body.entry()
Proprioception.entry()
Body.update()
Proprioception.update()

step:entry()
print('training rmp...')
step:learn_torso_orbit(xdata, tdata)
step:save_parameters(PARAMETER_FILE)
print('done')
step:set_support_foot('l')
step:set_nominal_initialization(true)
step:set_velocity(velocity)
step:initialize()

local q_legs  = step:get_configuration()
local v_torso = step:get_torso_state()[2]

-- initialize stance
--------------------------------------------------------------------------------
if (INIT_STANCE) then
  print('initializing stance...')
  Body.set_simulator_torso_frame(Transform.pose6D{0, 0, 100})

  while Body.get_time() < 2 do
    dcm:set_joint_position(q_legs, 'legs')
    Body.update()
  end

  Body.set_simulator_torso_frame(Transform.pose6D{0, 0, 0.558})
  while true do
    Body.update()
  end
end

-- run step controller
--------------------------------------------------------------------------------
print('updating step controller...')

step:initialize_simulator_state(0.2)
step:set_support_foot('l')

while true do

  step:start()
  while (step:is_active()) do
    Body.update()
    Proprioception.update()
    step:update()
    if (SLOW_MO) then
      unix.usleep(1e4)
    end
  end

  if (not LOOP) then
    break
  else
    if (step:get_support_foot() == 'l') then
      step:set_support_foot('r')
    else
      step:set_support_foot('l')
    end
    --step:set_nominal_initialization(false)
  end
end

print('done')

-- save step parameters and exit
--------------------------------------------------------------------------------
step:exit()
Proprioception.exit()
Body.exit()
