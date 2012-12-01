require('lcm')
require('lcm_rpc_request_t')

local userdata = "useful_userdata"

function rpc_request_callback(channel, msg, userdata)
  print(channel)
  print('client_id',   msg.client_id)
  print('request_id',  msg.request_id)
  print('eval_string', msg.eval_string)
  print('synchronous', msg.synchronous)
  print('userdata',    userdata)
end

rpc_server = lcm.new()
rpc_server:rpc_request_t_subscribe("EXAMPLE", rpc_request_callback, userdata)

while (true) do
  rpc_server:handle()
end
