require('zmq')
require('cmsgpack')

Platform = {}

local context = nil
local time_socket = nil
local time_poller = nil 
local time_endpoint = 'tcp://127.0.0.1:12000'

local time = 0
local time_step = 0
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
  time_poller = zmq.ZMQ_Poller(1)
  time_poller:add(time_socket, zmq.POLLIN)
  Platform.update()
end

function Platform.update()
  local count = time_poller:poll(-1)
  while (count > 0) do
    local msg = time_socket:recv()
    if msg then
      local time_t = cmsgpack.unpack(msg:sub(5))
      if (type(time_t) == 'table') then
	time, time_step = unpack(time_t)
      end
    end
    update_rate = 0.1*(1/time_step) + 0.9*update_rate
    count = time_poller:poll(0)
  end
end

function Platform.exit()
  time_socket:close()
  context:term()
end

return Platform
