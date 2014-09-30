assert(Config, 'Need a pre-existing Config table!')

local vector = require'vector'

Config.sensors = {
  'position', 'velocity', 'torque',
}
Config.actuators = {
  'command_position',
}

Config.nJoint = 5

Config.servo = {
  min_rad = DEG_TO_RAD * vector.new({-165, -57, -142, -95, -150}),
  max_rad = DEG_TO_RAD * vector.new({165, 89, 142, 95, 150}),
  offset  = DEG_TO_RAD * vector.new({-167, -58, -143, -97, -168}),
  direction = vector.new{1, -1, 1, -1, 1},
}
assert(#Config.servo.min_rad==Config.nJoint, 'Bad servo min_rad!')
assert(#Config.servo.max_rad==Config.nJoint, 'Bad servo min_rad!')
assert(#Config.servo.direction==Config.nJoint, 'Bad servo direction!')
assert(#Config.servo.offset==Config.nJoint, 'Bad servo offsets!')

---------------
-- Keyframes --
---------------
Config.km = {}

return Config
