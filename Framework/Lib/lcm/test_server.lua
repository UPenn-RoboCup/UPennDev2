require('lcm')
require('lcm_rpc_t')

function rpc_callback(channel, msg)
  print(channel)
  print('process_id', msg.process_id)
  print('request_id', msg.request_id)
  print('eval_string', msg.eval_string)
end

rpc_server = lcm.new()
rpc_server:rpc_t_subscribe("EXAMPLE", rpc_callback)

while (true) do
  rpc_server:handle()
end
