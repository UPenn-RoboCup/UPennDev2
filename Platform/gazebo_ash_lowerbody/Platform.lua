require('zmq')
require('cmsgpack')

Platform = {}

local context = nil
local time_socket = nil
local time_poller = nil 
local time_endpoint = 'tcp://127.0.0.1:12000'

local time_step = 0.002
local time_past = 0
local time_current = 0
local update_rate = 0

function Platform.get_time()
  -- get current time
  return time_current
end

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
  context = zmq.init(1)
  time_socket = context:socket(zmq.SUB)
  time_socket:connect(time_endpoint)
  time_socket:setopt(zmq.SUBSCRIBE, 'time')
  time_poller = zmq.ZMQ_Poller(1)
  time_poller:add(time_socket, zmq.POLLIN)
  Platform.update()
end

function Platform.update()
  -- synchronize thread with simulator time publisher
  while (time_current - time_past < time_step) do
    local count = time_poller:poll(-1)
    while (count > 0) do
      local msg = time_socket:recv()
      if msg then
	local time_t = cmsgpack.unpack(msg:sub(5))
	if (type(time_t) == 'number') then
	  time_current = time_t
        end
      end
      count = time_poller:poll(0)
    end
  end
  update_rate = 0.1/(time_current - time_past) + 0.9*update_rate
  time_past = time_current
end

function Platform.exit()
  time_socket:close()
  context:term()
end

return Platform
