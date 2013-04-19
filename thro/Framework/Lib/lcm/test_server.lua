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

local userdata = "useful_userdata"

function shm_callback(channel, msg, userdata)
  print(channel)
  for k, v in pairs(msg.position) do
    print(k ,v)
  end
  print('request_id',  msg.request_id)
  print('nbytes',      msg.nbytes)
  print('eval_string', msg.eval_string)
  print('id', msg.id)
  for k, v in pairs(msg.vel) do
    print(k, v)
  end
  print('userdata',    userdata)
end

rpc_server = lcm.new()
rpc_server:shm_t_subscribe("EXAMPLE", shm_callback, userdata)

while (true) do
  rpc_server:handle()
end
