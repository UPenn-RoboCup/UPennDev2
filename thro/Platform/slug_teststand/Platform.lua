require('unix')

Platform = {}

local t0 = unix.time()
local time_step = 0.002
local update_rate = 0

Platform.get_time = unix.time

function Platform.set_time_step(t)
  -- set desired time step
  assert(t > 0, 'time_step must be positive')
  time_step = t
end

function Platform.get_time_step()
  -- get desired time step
  return time_step
end

function Platform.set_update_rate(f)
  -- set desired update rate
  assert(f > 0, 'update_rate must be positive')
  time_step = 1/f
end

function Platform.get_update_rate()
  -- get actual update rate
  return update_rate
end

function Platform.reset_simulator()
end

function Platform.reset_simulator_physics()
end

function Platform.set_simulator_torso_frame(frame)
end

function Platform.set_simulator_torso_twist(twist)
end

function Platform.entry()
end

function Platform.update()
  local t = unix.time()
  local dt = t - t0

  -- regulate update rate
  if (dt < time_step) then
    unix.usleep((time_step - dt)*1e6)
    t = unix.time()
    dt = t - t0
  end

  t0 = t
  update_rate = 0.1*(1/dt) + 0.9*update_rate
end

function Platform.exit()
end

return Platform
