local state = {}
state._NAME = ...
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

-- Running estimate of where the legs are
local qLLeg, qRLeg, qWaist

function state.entry()
  print(state._NAME..' Entry')

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  Body.enable_read'lleg'
  Body.enable_read'rleg'

  if Config.walk.legBias then
    mcm.set_leg_bias(Config.walk.legBias)
    print("BIAS SET:",unpack(Config.walk.legBias))
  end

  --Set up initial walk parameters
  mcm.set_walk_tStep(Config.walk.tStep)
  mcm.set_walk_tZmp(Config.walk.tZmp)
  mcm.set_walk_bodyHeight(Config.walk.bodyHeight)
  mcm.set_walk_stepHeight(Config.walk.stepHeight)
  mcm.set_walk_torsoX(Config.walk.torsoX)
  mcm.set_walk_footY(Config.walk.footY)
  mcm.set_walk_supportX(Config.walk.supportX)
  mcm.set_walk_supportY(Config.walk.supportY)
  mcm.set_walk_hipRollCompensation(Config.walk.hipRollCompensation)

  mcm.set_motion_state(0)
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
--  print(state._NAME..' Update')
  -- Get the time of update
  local t = Body.get_time()

  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  if t - t_entry > timeout then return'timeout' end

end

function state.exit()
  print(state._NAME..' Exit' ) 
  -- Torque on the motors
  Body.set_waist_torque_enable(1)

  if Config.torque_legs then
    Body.set_lleg_torque_enable(1)
    Body.set_rleg_torque_enable(1)
  end
end

return state
