dofile('../include.lua')

require('rpc')
require('unix')

function my_remote_function(x)
  return math.cos(x)
end

-- initialize server
server = rpc.server.new('EXAMPLE')

-- handle rpc requests
while true do
  unix.usleep(1E3)
  io.stderr:write('.')
  server:update()
end
