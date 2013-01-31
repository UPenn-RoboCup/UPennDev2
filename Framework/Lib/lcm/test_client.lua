local function shell(command)
  local pipe = io.popen(command, 'r')
  local result = pipe:read('*a')
  pipe:close()
  return result
end

-- get dynamic lib suffix
local uname = shell('uname') 
if string.match(uname, 'Darwin') then
  csuffix = 'dylib'
else
  csuffix = 'so'
end

package.cpath = "./?."..csuffix..";"..package.cpath

require('lcm')
require('thor_rpc_request_t')

local msg = {
  client_id = "uuid",
  request_id = 456,
  eval_string = "walk.stop",
  synchronous = true
}

rpc_client = lcm.new()
rpc_client:rpc_request_t_publish("EXAMPLE", msg)
