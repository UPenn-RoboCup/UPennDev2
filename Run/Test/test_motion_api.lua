dofile('../include.lua')

require('rpc')
require('zmq')
require('unix')

local rpc_endpoint = 'tcp://localhost:12000'
local context = zmq.init()
local mm = rpc.client.new(rpc_endpoint, context)

function test_walk_api()
  -- enable walk controller
  print('Set locomotion state to walk...')
  assert(mm:call('Locomotion:set_state', 'walk'))

  -- start walking
  print('Start walking...')
  assert(mm:call('walk:start'))

  -- block until robot is walking
  while (true) do
    local status, walk_active = mm:call('walk:is_active')
    if (walk_active) then
      break
    end
    unix.usleep(1000)
  end

  -- adjust walking velocity (vx, vy, va)
  print('set velocity : 0  0.02  0')
  assert(mm:call('walk:set_velocity', 0, 0.02, 0))
  unix.sleep(3)

  print('set velocity : -0.02  0  0')
  assert(mm:call('walk:set_velocity', -0.02, 0, 0))
  unix.sleep(3)

  print('set velocity : 0  -0.02  0')
  assert(mm:call('walk:set_velocity', 0, -0.02, 0))
  unix.sleep(3)

  print('set velocity : 0.02  0  0')
  assert(mm:call('walk:set_velocity', 0.02, 0, 0))
  unix.sleep(3)

  print('set velocity : 0  0  0.03')
  assert(mm:call('walk:set_velocity', 0, 0, 0.03))
  unix.sleep(3)

  print('set velocity : 0  0  -0.03')
  assert(mm:call('walk:set_velocity', 0, 0, -0.03))
  unix.sleep(3)

  -- stop walking
  print('stop walking')
  assert(mm:call('walk:stop'))
end

print('        Test walk api')
print('*******************************')
print('Connect to motion manager...')

-- connect to motion manager and test api
assert(mm:connect(1))
mm:set_timeout(0.1)
test_walk_api()

-- cleanup
mm:close()
context:term()
