dofile('../include.lua')

require('rpc')

-- initialize client
client = rpc.new_client('EXAMPLE')
client:set_timeout(0.5)
assert(client:connect(1))

for k in pairs(client:get_dictionary()) do 
  print(k)
end

-- make a non-blocking remote prodedure call
client:eval('print', 'HELLO REMOTE WORLD')

-- make a blocking remote procedure call
status, result = client:call('my_remote_function', 0)

print('')
print('return status :', status)
print('return value  :', result)
