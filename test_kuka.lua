dofile'fiddle.lua'
local K = Body.Kinematics
--local T = require'Transform'
local T = require'libTransform'
local getch = require'getch'


-- Translate the end effector
local trans_arm = {
  u = T.trans(0,0,.01),
  m = T.trans(0,0,-.01),
  i = T.trans(.01,0,0),
  [','] = T.trans(-.01,0,0),
  j = T.trans(0,.01,0),
  l = T.trans(0,-.01,0),
}

-- Rotate (locally) the end effector
local rot_arm = {
  [']'] = T.rotY(.1),
  ['['] = T.rotY(-.1),
  ["'"] = T.rotZ(.1),
  [';'] = T.rotZ(-.1),
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
  
  local tmp = K.test()
  print('tmp',tmp)

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
  
  if trans_arm[char] then
    local d_tr = trans_arm[char]
    local qArm = Body.get_position()
    local fk = K.forward_arm(qArm)
    if T.copy then fk = T.copy(fk) end
    local desired_tr = d_tr * fk
    local iqArm = vector.new(K.inverse_arm(desired_tr))
    Body.set_command_position(iqArm)
    return
  elseif rot_arm[char] then
    local d_tr = rot_arm[char]
    local qArm = Body.get_position()
    local fk = K.forward_arm(qArm)
    if T.copy then fk = T.copy(fk) end
    local desired_tr = T.local_extrinsic_rot(fk,d_tr)
    local iqArm = vector.new(K.inverse_arm(desired_tr))
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
    -- Debug
    --[[
    local fk = K.forward_arm(qCmd)
    --print('fk1',fk)
    if T.copy then fk = T.copy(fk) end
    --print('fk2',fk)
    local p6D = T.position6D(fk)
    local fArm = vector.new(p6D)
    print('\nDirect | fArm',fArm)
    --local zyz = T.to_zyz(fk)
    --print('zyz',zyz)
    --print('qArm',qArm)
    --print('qCmd',qCmd)
    --]]
  elseif char=='-' then
    local qArm = Body.get_position()
    local qCmd = qArm - delta_q
    Body.set_command_position(qCmd)
    -- Debug
    --[[
    local fk = K.forward_arm(qCmd)
    if T.copy then fk = T.copy(fk) end
    local fArm = vector.new(T.position6D(fk))
    print('\nDirect | fArm',fArm)
    --local zyz = T.to_zyz(fk)
    --print('zyz',zyz)
    --print('qArm',qArm)
    --print('qCmd',qCmd)
    --]]
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
