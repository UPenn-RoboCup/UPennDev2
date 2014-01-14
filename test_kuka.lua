dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'Transform'
local getch = require'getch'

local move = {
  u = vector.new{0,0,.01,0,0,0},
  m = vector.new{0,0,-.01,0,0,0},
  i = vector.new{.01,0,0,0,0,0},
  [','] = vector.new{-.01,0,0,0,0,0},
  j = vector.new{0,.01,0,0,0,0},
  l = vector.new{0,-.01,0,0,0,0},
  ['='] = vector.new{0,0,0,0,.01,0},
  ['-'] = vector.new{0,0,0,0,-.01,0},
}

local qArm = Body.get_position()
local cur_arm = vector.new(T.position6D(K.forward_arm(qArm)))
print('cur_arm',cur_arm)

local function process_keycode(keycode,t_diff)
  local char = string.char(keycode)
  local char_lower = string.lower(char)
  
  if move[char] then
    print('cur_arm',cur_arm)
    local desired = cur_arm + move[char]
    print('desired',desired)
    local iqArm = vector.new(K.inverse_arm(desired))
    --print('iqArm',iqArm)
    Body.set_command_position(iqArm)
    cur_arm = desired
    return
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