lcm = require('lualcm')
require('lcm_rpc_t')

local rpc_request = {
  process_id = 124,
  request_id = 456,
  eval_string = "walk.stop()"
}
rpc_request.nbyyes = #rpc_request.eval_string

rpc_client = lcm.lcm_create(NULL)
lcm_rpc_t.lcm_rpc_t_publish(rpc_client, "EXAMPLE", rpc_request)
