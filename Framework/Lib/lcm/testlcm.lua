require('lcm')
--require('lcm_rpc_t')

--[[
local rpc_request = {
  process_id = 124,
  request_id = 456,
  eval_string = "walk.stop()"
}
rpc_request.nbyyes = #rpc_request.eval_string

rpc_client = lcm.lcm_create(NULL)
lcm_rpc_t.lcm_rpc_t_publish(rpc_client, "EXAMPLE", rpc_request)
--]]

--[[
for k,v in pairs(lcm) do
  if type(v) == 'function' then
    print(k, v)
  end
end
--]]

-- Set multicast TTL to 1 to limit packets published to local network
lc = lcm.lcm_create('udpm://239.255.76.67:7667?ttl=1')
if (lc == 0) then
  print("lcm create error");
  return 1;
else
  print(type(lc))
end

--[[
while (true) do
  lcm.lcm_handle(lc);
end
--]]

lcm.lcm_destroy(lc)

