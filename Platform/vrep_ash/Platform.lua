require('unix')

Platform = {}

local zmq_context
local frame_update_socket
local sim_time = 0

local t0 = unix.time()
local time_step = 0.002
local update_rate = 0

function Platform.get_time()
  return sim_time
end

function Platform.set_time_step(t)
  time_step = t
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
  zmq_context = zmq.init(1)
  frame_update_socket = zmq_context:socket(zmq.SUB)
  frame_update_socket:connect("tcp://127.0.0.1:12000")
  frame_update_socket:setopt(zmq.SUBSCRIBE, "f")
end

function Platform.update()
  local t = unix.time()
  local dt = t - t0

  local msg = frame_update_socket:recv()
  if msg ~= 0 then
    sim_time = tonumber(msg:sub(2))
  end

  t0 = t
  update_rate = 0.1*(1/dt) + 0.9*update_rate
end

function Platform.exit()
  frame_update_socket:close()
  zmq_context:term()
end

return Platform
