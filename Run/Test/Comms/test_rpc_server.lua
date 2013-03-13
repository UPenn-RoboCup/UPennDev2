dofile('../../include.lua')

require('rpc')
require('zmq')
require('unix')

function my_remote_function(x)
  return math.cos(x)
end

-- initialize server
server = rpc.server.new('tcp://127.0.0.1:5555')
server:set_timeout(0)

-- handle rpc requests
while true do
  unix.usleep(1E3)
  io.stderr:write('.')
  server:update()
end

server:close()
