dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'Transform'
local getch = require'getch'

local move_arm = {
  u = vector.new{0,0,.01,0,0,0},
  m = vector.new{0,0,-.01,0,0,0},
  i = vector.new{.01,0,0,0,0,0},
  [','] = vector.new{-.01,0,0,0,0,0},
  j = vector.new{0,.01,0,0,0,0},
  l = vector.new{0,-.01,0,0,0,0},
  ['='] = vector.new{0,0,0,0,.1,0},
  ['-'] = vector.new{0,0,0,0,-.1,0},
}

local move_base = {
  w = vector.new{.01,0,0},
  x = vector.new{-.01,0,0},
  a = vector.new{0,.01,0},
  d = vector.new{0,-.01,0},
  q = vector.new{0,0,.01},
  e = vector.new{0,0,-.01},
}

local qArm = Body.get_position()
local cur_arm = vector.new(T.position6D(K.forward_arm(qArm)))
print('cur_arm',cur_arm)

local function process_keycode(keycode,t_diff)
  local char = string.char(keycode)
  local char_lower = string.lower(char)
  
  if move_arm[char] then
    print('cur_arm',cur_arm)
    local desired = cur_arm + move_arm[char]
    print('desired',desired)
    local iqArm = vector.new(K.inverse_arm(desired))
    --print('iqArm',iqArm)
    Body.set_command_position(iqArm)
    cur_arm = desired
    return
  end
  
  if move_base[char] then
    local cur_vel = mcm.get_walk_vel()
    local desired = cur_vel + move_base[char]
    print('Vel',desired)
    mcm.set_walk_vel(desired)
  elseif char=='s' then
    print('Vel',vector.zeros(3))
    mcm.set_walk_vel{0,0,0}
  end
  
end

------------
-- Start processing
io.flush()
local t0 = unix.time()
while true do
  
  -- Grab the keyboard character
  local keycode = getch.block()
  
  -- Measure the timing
  local t = unix.time()
  local t_diff = t - t0
  t0 = t
  
  -- Process the character
  process_keycode(keycode,t_diff)
    
end