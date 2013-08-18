dofile'include.lua'
local simple_ipc = require'simple_ipc'
local util = require'util'
local mp = require'msgpack'
local quaternion = require'quaternion'

-- Listen for mesh requests on both zmq and udp
local zmq_send_ch = simple_ipc.new_publisher'from_operator'
local zmq_recv_ch = simple_ipc.new_subscriber'to_operator'
local zmq_recv_tk_ch = simple_ipc.new_subscriber'from_telekinesis'

-- Access necessary shared memory segments
require'tkcm'

local function p_telekinesis()
  -- TODO: This should be non blocking...
  local current = zmq_recv_tk_ch:receive()
  local msg = {}
  msg.pos = tkcm.get_drill_position()
  msg.q = tkcm.get_drill_orientation()
  msg.t   = tkcm.get_drill_t()
  msg.tool=current
  zmq_send_ch:send( mp.pack(msg) )
end

-- Dictionary of forwarding requests
local forwarder = {
  ['telekinesis'] = p_telekinesis
}

-- Lookup table based on the name of the request
local function process_zmq()
  local cmd, has_more = zmq_recv_ch:receive()
  local command = mp.unpack(cmd)
  if not command.type then return end
  local info_str = string.format('%s',command.type)
  -- Try to execute the command
  local my_request_func = forwarder[command.type]
  if my_request_func then
    info_str = util.color(info_str,'green')
    my_request_func()
  else
    info_str = util.color(info_str,'red')
  end
  print('Operator Command: '..info_str..'.')
end

-- Set up callback on ZMQ
local wait_channels = {}
if zmq_recv_ch then
  zmq_recv_ch.callback = process_zmq
  table.insert(wait_channels,zmq_recv_ch)
end

if zmq_recv_tk_ch then
  zmq_recv_tk_ch.callback = p_telekinesis
  table.insert(wait_channels,zmq_recv_tk_ch)
end

-- Begin to poll on ZeroMQ requests
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
local channel_timeout = 1e3 * 1/30 -- milliseconds
while true do
  local ret = channel_poll:poll(channel_timeout)
end