dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'libTransform'
local getch = require'getch'

-- Translate the end effector
local ds = 0.005
local trans_arm = {
  u = T.trans(0,0,ds),
  m = T.trans(0,0,-ds),
  i = T.trans(ds,0,0),
  [','] = T.trans(-ds,0,0),
  j = T.trans(0,ds,0),
  l = T.trans(0,-ds,0),
}

-- Rotate (locally) the end effector
local dr = 0.05
local rot_arm = {
  [']'] = T.rotY(dr),
  ['['] = T.rotY(-dr),
  ["'"] = T.rotZ(dr),
  [';'] = T.rotZ(-dr),
}

local dv = 0.01
local move_base = {
  w = vector.new{dv,0,0},
  x = vector.new{-dv,0,0},
  a = vector.new{0,dv,0},
  d = vector.new{0,-dv,0},
  q = vector.new{0,0,.01},
  e = vector.new{0,0,-.01},
}

local dq = 0.05
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
  
  if trans_arm[char] then
    local d_tr = trans_arm[char]
    local qArm = Body.get_position()
    local fk = K.forward_arm(qArm)
    local desired_tr = d_tr * fk
    --
    print('\Rotate to')
    print(T.tostring(desired_tr))
    --
    local iqArm = vector.new(K.inverse_arm(desired_tr))
    Body.set_command_position(iqArm)
    return
  elseif rot_arm[char] then
    local d_tr = rot_arm[char]
    local qArm = Body.get_position()
    local fk = K.forward_arm(qArm)
    local desired_tr = T.local_extrinsic_rot(fk,d_tr)
    --
    print('\Rotate to')
    print(T.tostring(desired_tr))
    --
    local iqArm = vector.new(K.inverse_arm(desired_tr))
    Body.set_command_position(iqArm)
    return
  end

  local num = tonumber(char)
  if num then
    delta_q = vector.zeros(5)
    delta_q[num] = dq
    return
  elseif char=='=' then
    local qArm = Body.get_position()
    local qCmd = qArm + delta_q
    Body.set_command_position(qCmd)
    print('\nDirect to',qCmd)
    return
  elseif char=='-' then
    local qArm = Body.get_position()
    local qCmd = qArm - delta_q
    Body.set_command_position(qCmd)
    print('\nDirect to',qCmd)
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
  
end

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
