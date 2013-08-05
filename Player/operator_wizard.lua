dofile'include.lua'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local udp_target = 'localhost'
local udp_port   = 43288
require'vcm'

-- Listen for mesh requests on both zmq and udp
local mesh_req_zmq = simple_ipc.new_subscriber('mesh_requests')
assert(mesh_req_zmq,"Bad zmq receiver!")
local mesh_req_udp = udp.new_receiver( udp_port )
assert(mesh_req_udp,"Bad udp receiver!")

local function mesh_request()
  print('Performing a mesh request...')
  vcm.set_chest_lidar_mesh_request(1)
end

-- Dictionary of forwarding requests
local forwarder = {
  ['mesh_request'] = mesh_request
}

local function process_zmq()
  local req,has_more = mesh_req_zmq:receive()
  local my_request_func = forwarder[req]
  if my_request_func then my_request_func() end
end

local function process_udp()
  print('there!')
end

-- Set up callback on ZMQ
mesh_req_zmq.callback = process_zmq

-- Set up callback on UDP
local mesh_req_udp_poll = {}
mesh_req_udp_poll.socket_handle = mesh_req_udp:descriptor()
mesh_req_udp_poll.callback      = process_udp

local wait_channels = {}
table.insert(wait_channels,mesh_req_zmq)
table.insert(wait_channels,mesh_req_udp_poll)
local channel_poll = simple_ipc.wait_on_channels( wait_channels )

channel_poll:start()