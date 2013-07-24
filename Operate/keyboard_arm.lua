-----------------------------------------------------------------
-- Keyboard Arm control
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

-- Libraries
local unix = require'unix'
local getch = require'getch'

-- Getting/Setting shared memory
require'jcm'

local debug = true
local current_joint = 1
local current_arm = 'left'

local min_joint, max_joint = 1, 6

local joint_names = {
  [1]='wrist',
  [2]='wrist',
  [3]='elbow',
  [4]='shoulder',
  [5]='shoulder',
  [6]='shoulder',
}

local joint_ids = {
  ['left'] = {
    [1]=1,
    [2]=2,
    [3]=3,
    [4]=4,
    [5]=5,
    [6]=6
    },
    ['right'] = {
      [1]=1,
      [2]=2,
      [3]=3,
      [4]=4,
      [5]=5,
      [6]=6
    }
  }

print('\n\n=========')
print(string.format('Starting on %s joint on the %s arm.',
joint_names[current_joint],current_arm))

-- Print Message helpers
local switch_msg = function()
  return string.format('Switched to %s joint on the %s arm.', 
  joint_names[current_joint],current_arm)
end

local change_msg = function(old,new)
  local inc_dec = 'Set'
  if new>old then inc_dec='Increased'
  elseif new<old then inc_dec='Decreased'
  end
  return string.format('%s arm | %s %s joint to %.3f', 
  current_arm,inc_dec,joint_names[current_joint],new)
end

local function get_joint()
  local joint_id = joint_ids[current_arm][current_joint]
  local joints = jcm.get_commanded_position()
  local current = joints[joint_id]
  return current
end

local function set_joint(val)
  assert(type(val)=='number','Bad set!')
  local joint_id = joint_ids[current_arm][current_joint]
  local joints = jcm.get_commanded_position()
  joints[joint_id] = val
  jcm.set_commanded_position(joints)
end

-- Character processing
local function process_character(key_code,key_char,key_char_lower)

  -- Number keys switch the joint on the arm chain
  -- Number 0 zeros the joint
  local switch_joint = tonumber(key_char)  
  if switch_joint then
    -- Zero the motor
    if switch_joint==0 then
      local current = get_joint()
      set_joint(0)
      return change_msg(current,0)
    end
    -- Check that the joint number is in range
    if switch_joint<min_joint or switch_joint>max_joint then
      return 'Joint '..switch_joint..' out of range!'
    end
    -- Switch to that joint
    current_joint = switch_joint
    return switch_msg()
  end
  
  -- Bracket keys switch arms
  if key_char=='[' then
    current_arm = 'left'
    return switch_msg()
  elseif key_char=='[' then
    current_arm = 'right'
    return switch_msg()
  end
  
  -- Bracket keys switch arms
  if key_char=='[' then
    current_arm = 'left'
    return switch_msg()
  elseif key_char=='[' then
    current_arm = 'right'
    return switch_msg()
  end
  
  -- +/- Increases and decreases
  if key_char=='-' then
    local current = get_joint()
    local new = current - .1
    set_joint(new)
    return change_msg(current,new)
  elseif key_char=='=' then -- +/= are the same key
    local current = get_joint()
    local new = current + .1
    set_joint(new)
    return change_msg(current,new)
  end
  
  -- Help and debugging
  if key_char=='h' then
    return'HELP'
  elseif key_char=='p' then
    return'Print arm status'
  end
  
  -- Default message
  return 'Nothing changes'
  
end

local t0 = unix.time()
while true do
  
  -- Grab the keyboard character
  local key_code = getch.block()
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)
  
  -- Process the character
  local msg = process_character(key_code,key_char,key_char_lower)
  
  -- Measure the timing
  local t = unix.time()
  local t_diff = t-t0
  t0 = t
  local fps = 1/t_diff
  
  -- Print debugging message
  if debug then
    print( string.format('\nKeyboard | Code: %d, Char: %s, Lower: %s',
    key_code,key_char,key_char_lower) )
    print('Response time:',t_diff)
  end
  
  -- Print result of the key press
  print(msg)
    
end