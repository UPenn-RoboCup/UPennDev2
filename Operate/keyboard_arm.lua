-----------------------------------------------------------------
-- Keyboard Arm control
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

-- Libraries
local unix = require'unix'
local getch = require'getch'

require'jcm'

local debug = true
local current_joint = 1
local current_arm = 'left'

local joint_names = {
  [1]='wrist',
  [2]='wrist',
  [3]='wrist',
  [4]='wrist',
  [5]='wrist',
  [6]='wrist',
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

-- Character processing
local function process_character(key_code,key_char,key_char_lower)

  -- Number keys switch the joint on the arm chain
  local switch_joint = tonumber(key_char)  
  if switch_joint then
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
    return change_msg(1,0)
  elseif key_char=='=' then -- +/= are the same key
    return change_msg(0,1)
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
  local fps = 1/t_diff
  t0 = t
  
  -- Print debugging message
  if debug then
    print( string.format('\nKeyboard | Code: %d, Char: %s, Lower: %s',
    key_code,key_char,key_char_lower) )
    print('Response time:',t_diff)
  end
  
  -- Print result of the key press
  print(msg)
    
end