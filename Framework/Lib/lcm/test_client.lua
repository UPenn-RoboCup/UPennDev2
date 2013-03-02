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
require('thor_shm_t')

local msg = {
  request_id = 456,
  nbytes     = 20,
  eval_string = "walk.stop",
  id = 345,
  position = {34,45, 32},
  vel = {0.54, 54.33},
}

rpc_client = lcm.new()
rpc_client:shm_t_publish("EXAMPLE", msg)
