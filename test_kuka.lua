dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'Transform'
local getch = require'getch'

local ik_arm = {
  u = vector.new{0,0,.01,0,0,0},
  m = vector.new{0,0,-.01,0,0,0},
  i = vector.new{.01,0,0,0,0,0},
  [','] = vector.new{-.01,0,0,0,0,0},
  j = vector.new{0,.01,0,0,0,0},
  l = vector.new{0,-.01,0,0,0,0},
  [']'] = vector.new{0,0,0,0,.1,0},
  ['['] = vector.new{0,0,0,0,-.1,0},
}

local move_base = {
  w = vector.new{.01,0,0},
  x = vector.new{-.01,0,0},
  a = vector.new{0,.01,0},
  d = vector.new{0,-.01,0},
  q = vector.new{0,0,.01},
  e = vector.new{0,0,-.01},
}

local direct_arm = {
  [1] = vector.new{.01,0,0,0,0},
  [2] = vector.new{.01,0,0,0,0},
  [3] = vector.new{.01,0,0,0,0},
  [4] = vector.new{.01,0,0,0,0},
  [5] = vector.new{.01,0,0,0,0},
}

local delta_q = vector.zeros(5)
local function process_keycode(keycode,t_diff)
  local char = string.char(keycode)
  local char_lower = string.lower(char)
  
  -- Open and close the gripper
  if char_lower=='g' then
    local cur_g = jcm.get_gripper_command_position()
    cur_g[1] = math.max(cur_g[1] - 0.0025,0)
    jcm.set_gripper_command_position(cur_g)
    print('New gripper',cur_g[1])
    return
  elseif char_lower=='h' then
    local cur_g = jcm.get_gripper_command_position()
    cur_g[1] = math.min(cur_g[1] + 0.0025,0.025)
    jcm.set_gripper_command_position(cur_g)
    print('New gripper',cur_g[1])
    return
  end
  
  -- Avoid any arm motions, since tailored for webots zero positions
  --if true then return end
  
  if ik_arm[char] then
    local qArm = Body.get_position()
    local sensed_arm = vector.new(T.position6D(K.forward_arm(qArm)))

    -- TODO: HACK: This should be done in the IK...
    local pitch = vector.sum(vector.slice(qArm,2,4))
    sensed_arm[4],sensed_arm[5],sensed_arm[6] = 0, pitch, 0
    -- END HACK

    local desired = sensed_arm + ik_arm[char]
    local iqArm = vector.new(K.inverse_arm(desired))
    --
    print('\nIK | desired',desired)
    --print('pitch',pitch)
    --print('sensed_arm',sensed_arm)
    --print('qArm',qArm)
    --print('iqArm',iqArm)
    --
    Body.set_command_position(iqArm)
    return
  end
  
  if move_base[char] then
    local cur_vel = mcm.get_walk_vel()
    local desired = cur_vel + move_base[char]
    print('\n\tBase |',desired)
    mcm.set_walk_vel(desired)
    return
  elseif char=='s' then
    local desired = vector.zeros(3)
    print('\n\tBase |',desired)
    mcm.set_walk_vel(desired)
    return
  end

  local num = tonumber(char)
  if num then
    delta_q = vector.zeros(5)
    delta_q[num] = .1
    return
  elseif char=='=' then
    local qArm = Body.get_position()
    local qCmd = qArm + delta_q
    Body.set_command_position(qCmd)
    local fk = K.forward_arm(qCmd)
    local fArm = vector.new(T.position6D(fk))
    print('\nDirect | fArm',fArm)
    --print('qArm',qArm)
    --print('qCmd',qCmd)
  elseif char=='-' then
    local qArm = Body.get_position()
    local qCmd = qArm - delta_q
    Body.set_command_position(qCmd)
    local fk = K.forward_arm(qCmd)
    local fArm = vector.new(T.position6D(fk))
    print('\nDirect | fArm',fArm)
    --print('qArm',qArm)
    --print('qCmd',qCmd)
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
