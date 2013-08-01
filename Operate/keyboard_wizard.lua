-----------------------------------------------------------------
-- Keyboard Wizard
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

--local is_debug = true

-- Libraries
local unix = require'unix'
local getch = require'getch'
local mp = require'msgpack'
local colors = require'colors'

-- Getting/Setting The Body
local Body = require'Body'

local current_joint = 1
local current_arm = 'larm'
-- Modes: direct, ik
local current_mode = 1
local mode_msg = {
  'direct',
  'inverse kinematics'
}
-- Arm with three fingers
local max_joint = #Body.parts['LArm']+3

-- Change in radians for each +/-
local DEG_TO_RAD = math.pi/180
local delta_joint = 1 * DEG_TO_RAD
local delta_ik = .01 -- meters

-- Keyframing
local keyframe_num = 0
local keyframe_file_num = 0
local function add_keyframe()
  keyframe_num = keyframe_num+1
end
local function save_keyframes()
  local filename = string.format('keyframe_%d.mp.raw',keyframe_file_num)
  keyframe_file_num = keyframe_file_num + 1
  keyframe_num = 0
  return filename
end

local function joint_name()
	local jName = 'Unknown'
	if current_arm=='larm' then
		if current_joint<7 then
			jName = Body.jointNames[Body.indexLArm+current_joint-1]
		else
			jName = 'finger '..current_joint-6
		end
	elseif current_arm=='rarm' then
		if current_joint<7 then
			jName = Body.jointNames[Body.indexRArm+current_joint-1]
		else
			jName = 'finger '..current_joint-6
		end
	end
	return jName
end

-- Joint access helpers
local function get_joint()
  if current_arm=='larm' then
    if current_joint<7 then
      return Body.get_larm_command_position(current_joint)
    else
      -- finger
      return Body.get_aux_command_position(current_joint-6)
    end
  else
    if current_joint<7 then
      return Body.get_rarm_command_position(current_joint)
    else
      -- finger
      return Body.get_aux_command_position(current_joint-3)
    end
  end
end

local function set_joint(val)
  if current_arm=='larm' then
    if current_joint<7 then
      Body.set_larm_command_position(val,current_joint)
    else
      -- finger
      Body.set_aux_command_position(val,current_joint-6)
    end
  else
    -- Right
    if current_joint<7 then
      Body.set_rarm_command_position(val,current_joint)
    else
      -- finger
      Body.set_aux_command_position(val,current_joint-3)
    end
  end
end


-- Print Message helpers
local switch_msg = function()
  local jName = joint_name()
  local sw = string.format('Switched to %s %s @ %.2f radians.', 
  current_arm, jName, get_joint() )
  return sw
end

local change_msg = function(old,new)
  local inc_dec = 'Set'
  if new>old then inc_dec='Increased'
  elseif new<old then inc_dec='Decreased'
  end
  return string.format('%s arm | %s %s to %.3f', 
  current_arm,inc_dec,joint_name(),new)
end

local function jangle_str(arm_name,arm_angles,finger_angles)
  local text = ''
  for i,v in ipairs(arm_angles) do 
    if i==current_joint then
      if current_arm==arm_name:lower() then
        text = text..' *'
      else
        text = text..'  '
      end
    else
      text = text..' '
    end
    text = text..string.format( '%6.3f', v )
  end
  for i,v in ipairs(finger_angles) do
    if i+6==current_joint then
      if current_arm==arm_name:lower() then
        text = text..' *'
      else
        text = text..'  '
      end
    else
      text = text..' '
    end
    text = text..string.format( '%6.3f', v )
  end
  return text
end

-- TODO: Add torque enabling
local function state_msg()

  -- Command
  local larm_cmd = Body.get_larm_command_position()
  local rarm_cmd = Body.get_rarm_command_position()
  local lfinger_cmd = Body.get_lgrip_command_position()
  local rfinger_cmd = Body.get_rgrip_command_position()

  -- Position
  local larm = Body.get_larm_position()
  local rarm = Body.get_rarm_position()
  local lfinger = Body.get_lgrip_position()
  local rfinger = Body.get_rgrip_position()
  
  -- Load
  local larm_load = Body.get_larm_load()
  local rarm_load = Body.get_rarm_load()
  local lfinger_load = Body.get_lgrip_load()
  local rfinger_load = Body.get_rgrip_load()
  
  -- Torque Enable
  local larm_en = Body.get_larm_torque_enable()
  local rarm_en = Body.get_rarm_torque_enable()
  local lfinger_en = Body.get_lgrip_torque_enable()
  local rfinger_en = Body.get_rgrip_torque_enable()
  
  -- Inverse Kinematics
  local pL = Body.get_forward_larm()
  local pR = Body.get_forward_rarm()
  
  -- Make the message
  local msg = colors.wrap('\nKeyboard Wizard\n','blue')
  msg = msg..'Current State\n'
  msg = msg..'Operating on '..current_arm..' '..joint_name()..' in radians'

  -- Add the IK processing
  msg = msg..string.format(
    '\nLeft  IK:\t(%.2f  %.2f  %.2f) (%.2f  %.2f  %.2f)',unpack(pL)
  )
  msg = msg..string.format(
    '\nRight IK:\t(%.2f  %.2f  %.2f) (%.2f  %.2f  %.2f)',unpack(pR)
  )
  -- Add the shared memory
  msg = msg..'\n\nLeft  pos\t'..jangle_str('larm', larm,lfinger)
  msg = msg..'\nRight pos\t'..jangle_str('rarm',rarm,rfinger)
  msg = msg..'\n\nLeft  cmd\t'..jangle_str('larm', larm_cmd,lfinger_cmd)
  msg = msg..'\nRight cmd\t'..jangle_str('rarm',rarm_cmd,rfinger_cmd)
  msg = msg..'\n\nLeft  load\t'..jangle_str('larm', larm_load,lfinger_load)
  msg = msg..'\nRight load\t'..jangle_str('rarm',rarm_load,rfinger_load)
  msg = msg..'\n\nLeft  enable\t'..jangle_str('larm', larm_en,lfinger_en)
  msg = msg..'\nRight enable\t'..jangle_str('rarm',rarm_en,rfinger_en)
  
  -- Return the message
  return msg
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
    if switch_joint<1 or switch_joint>max_joint then
      return 'Joint '..switch_joint..' out of range!'
    end
    -- Switch to that joint
    current_joint = switch_joint
    return switch_msg()
  end
  
  -- Bracket keys switch arms
  if key_char=='[' then
    current_arm = 'larm'
    return switch_msg()
  elseif key_char==']' then
    current_arm = 'rarm'
    return switch_msg()
  end
  
  -- +/- Increases and decreases
  if key_char=='-' then
    local current = get_joint()
    local new = current - delta_joint
    set_joint(new)
    return change_msg(current,new)
  elseif key_char=='=' then -- +/= are the same key
    local current = get_joint()
    local new = current + delta_joint
    set_joint(new)
    return change_msg(current,new)
  end
  
  -- Keyframe management
  if key_char_lower=='k' then
    local keyframe = add_keyframe()
    return string.format('Added keyframe %d!',keyframe_num)
  elseif key_char_lower=='v' then
    local name = save_keyframes()
    return string.format('Saved keyframe file %s!',name)
  end
  
  -- Help and is_debugging
  if key_char_lower=='h' then
    return'HELP'
  elseif key_char_lower=='p' then
    return 'Printing the state'
  end
  
  -- Control modes
  -- TODO: Why need to change modes at all?
  if key_char_lower=='m' then
    current_mode = current_mode + 1
    if current_mode>#mode_msg then current_mode=1 end
    return string.format('Switched to %s mode',mode_msg[current_mode])
  end
  
  -- IK Changes with wasd
  local iku = nil
  if key_char_lower=='w' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({1,0,0, 0,0,0}))
  elseif key_char_lower=='s' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({-1,0,0, 0,0,0}))
  elseif key_char_lower=='a' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({0,1,0, 0,0,0}))
  elseif key_char_lower=='d' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({0,-1,0, 0,0,0}))
  elseif key_char_lower=='q' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({0,0,1, 0,0,0}))
  elseif key_char_lower=='z' then
    iku=Body['set_inverse_'..current_arm](delta_ik*vector.new({0,0,-1, 0,0,0}))
  end
  if iku then return'updated ik!' end

  -- Default message
  return 'Nothing changes'
  
end

------------
-- Start processing
os.execute("clear")
io.write( '\n\n',state_msg() )
io.flush()
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
  
  -- Print is_debugging message
  if is_debug then
    print( string.format('\nKeyboard | Code: %d, Char: %s, Lower: %s',
    key_code,key_char,key_char_lower) )
    print('Response time:',t_diff)
  end
  
  -- http://lua-users.org/lists/lua-l/2009-12/msg00937.html
  -- Print result of the key press
  os.execute("clear")
  print( state_msg() )
  print()
  print(colors.wrap(msg,'yellow'))
    
end
