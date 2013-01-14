dofile('../include.lua')

require('Body')
require('Proprioception')
require('trajectory')
require('Transform')
require('stepRMP')
require('gnuplot')
require('dcm')

local LOOP = true
local SLOW_MO = false
local PI2_RESULTS_FILE = 'results125.lua'
local PARAMETER_LOAD_FILE = '../../Data/parameters_stepRMP_WebotsASH_train125.lua'
local PARAMETER_SAVE_FILE = '../../Data/parameters_stepRMP_WebotsASH_learn125.lua'
local pi2_dimensions = {1, 2}
local velocity = {0.125, 0, 0}

-- initialize step controller
--------------------------------------------------------------------------------
Body.entry()
Proprioception.entry()
Body.update()
Proprioception.update()

local pi2_results = dofile(PI2_RESULTS_FILE)

step:entry()
step:load_parameters(PARAMETER_LOAD_FILE)
for i = 1, #pi2_dimensions do
  step:set_rmp_parameters(pi2_results.final_parameters[i], pi2_dimensions[i])
end
step:set_nominal_initialization(true)
step:set_velocity(velocity)
step:initialize_simulator_state(0.2)
step:save_parameters(PARAMETER_SAVE_FILE)
step:set_support_foot('l')

-- run step controller
--------------------------------------------------------------------------------
print('updating step controller...')


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
  end
end

print('done')

-- save step parameters and exit
--------------------------------------------------------------------------------
step:exit()
Proprioception.exit()
Body.exit()
