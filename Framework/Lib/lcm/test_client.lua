require('lcm')
require('lcm_rpc_t')

local msg = {
  process_id = 123,
  request_id = 456,
  eval_string = "walk.stop()",
  synchronous = true
}
msg.eval_nbytes = #msg.eval_string

rpc_client = lcm.new()
rpc_client:rpc_t_publish("EXAMPLE", msg)
