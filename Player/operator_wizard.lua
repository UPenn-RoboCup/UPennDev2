dofile'include.lua'
local simple_ipc = require'simple_ipc'
local util = require'util'
local mp = require'msgpack'

-- Listen for mesh requests on both zmq and udp
local zmq_send_ch = simple_ipc.new_publisher'to_operator'
local zmq_recv_ch = simple_ipc.new_subscriber'from_operator'

-- Access necessary shared memory segments
require'vcm'

local function p_spacemouse()
  print('Performing a spacemouse request...')
end

-- Dictionary of forwarding requests
local forwarder = {
  ['spacemouse'] = p_spacemouse
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

-- Begin to poll on ZeroMQ requests
local wait_channels = {}
if zmq_recv_ch then
  -- Set up callback on ZMQ
  print('Establishing callback.')
  zmq_recv_ch.callback = process_zmq
  table.insert(wait_channels,zmq_recv_ch)
end
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
channel_poll:start()