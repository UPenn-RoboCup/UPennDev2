require('lcm')
require('lcm_rpc_t')

local rpc_request = {
  process_id = 123,
  request_id = 456,
  eval_string = "walk.stop()" 
}
rpc_request.nbytes = #rpc_request.eval_string 

rpc_client = lcm.new()
rpc_client:rpc_t_publish("EXAMPLE", rpc_request)
