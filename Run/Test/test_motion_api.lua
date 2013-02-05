dofile('../include.lua')

require('rpc')
require('unix')

function test_walk_api()
  print('        Test walk api')
  print('*******************************')

  -- enable walk controller
  print('start walking')
  assert(mm:call('Locomotion:set_state', 'walk'))

  -- start walking
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

-- create an rpc client to communicate with the motion manager
mm = rpc.client.new('MOTION')
mm:connect()

-- test walk api 
test_walk_api()
