require('unix')

Platform = {}

local context = nil
local time_socket = nil
local time_endpoint = 'tcp://127.0.0.1:12000'

local time = 0
local t0 = unix.time()
local time_step = 0.002
local update_rate = 0

function Platform.get_time()
  return time
end

function Platform.set_time_step(t)
end

function Platform.get_time_step()
  return time_step
end

function Platform.get_update_rate()
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
  context = zmq.init(1)
  time_socket = context:socket(zmq.SUB)
  time_socket:connect(time_endpoint)
  time_socket:setopt(zmq.SUBSCRIBE, 'time')
end

function Platform.update()
  local t = unix.time()
  local dt = t - t0

  local msg = time_socket:recv()
  if msg ~= 0 then
    time = tonumber(msg:sub(5))
  end

  t0 = t
  update_rate = 0.1*(1/dt) + 0.9*update_rate
end

function Platform.exit()
  time_socket:close()
  context:term()
end

return Platform
