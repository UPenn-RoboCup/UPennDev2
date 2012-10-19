require('unix')

Body = {}

local t0 = unix.time()
local time_step = 0.002
local update_rate = 0

Body.get_time = unix.time

function Body.set_time_step(t)
  time_step = t
end

function Body.get_time_step()
  return time_step 
end

function Body.get_update_rate()
  return update_rate
end

function Body.entry()
end

function Body.update()
  local t = unix.time()
  local dt = t - t0
  t0 = t
  -- regulate update rate
  if (dt < time_step) then
    unix.usleep((time_step - dt)*1e6)
    dt = time_step
  end
  update_rate = 0.1*(1/dt) + 0.9*update_rate
end

function Body.exit()
end

return Body
