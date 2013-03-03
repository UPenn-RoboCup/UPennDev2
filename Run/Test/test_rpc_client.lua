dofile('../include.lua')

require('rpc')
require('zmq')


-- initialize client
local context = zmq.init()
client = rpc.client.new('tcp://localhost:5555', context)
client:set_timeout(0.5)
assert(client:connect(1))

status = client:call('io.stderr:write', 'HELLO REMOTE WORLD')
status, result = client:call('my_remote_function', 0)

print('')
print('return status :', status)
print('return value  :', result)

client:close()
context:term()
