require('lcm')
require('lcm_rpc_t')

local userdata = "some useful data"

function rpc_callback(channel, msg, userdata)
  print(channel)
  print('process_id', msg.process_id)
  print('request_id', msg.request_id)
  print('eval_string', msg.eval_string)
  print('synchronous', msg.synchronous)
  print('userdata', userdata)
end

rpc_server = lcm.new()
rpc_server:rpc_t_subscribe("EXAMPLE", rpc_callback, userdata)

while (true) do
  rpc_server:handle()
end
