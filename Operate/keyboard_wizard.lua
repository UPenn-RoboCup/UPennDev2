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

-- Getting/Setting The Body
local Body = require'Body'

local current_joint = 1
local current_arm = 'left'
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
	if current_arm=='left' then
		if current_joint<7 then
			jName = Body.jointNames[Body.indexLArm+current_joint-1]
		else
			jName = 'finger '..current_joint-6
		end
	elseif current_arm=='right' then
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
  if current_arm=='left' then
    if current_joint<7 then
      return Body.get_larm_command(current_joint)
    else
      -- finger
      return Body.get_aux_command(current_joint-6)
    end
  else
    if current_joint<7 then
      return Body.get_rarm_command(current_joint)
    else
      -- finger
      return Body.get_aux_command(current_joint-3)
    end
  end
end

local function set_joint(val)
  if current_arm=='left' then
    if current_joint<7 then
      Body.set_larm_command(val,current_joint)
    else
      -- finger
      Body.set_aux_command(val,current_joint-6)
    end
  else
    -- Right
    if current_joint<7 then
      Body.set_rarm_command(val,current_joint)
    else
      -- finger
      Body.set_aux_command(val,current_joint-3)
    end
  end
end

-- Change the joints via IK
local function ik_change(dx,dy,dz)
  -- Perform FK to get current coordinates
  -- Solve for the new set of coordinates
  -- For now, just go there...
  -- Other layers should perform safety checks
  local target = nil
  local p = nil
  if current_arm=='left' then
    p = Body.get_forward_larm()
  else
    p = Body.get_forward_rarm()
  end
  
  print('\np', unpack(p) )
  
  -- Increment the position
  p[1] = p[1] + dx
  p[2] = p[2] + dy
  p[3] = p[3] + dz
  
  print('\nnp', unpack(p) )
  
  if current_arm=='left' then
    target = Body.get_inverse_larm(p)
    if target then Body.set_larm_command(target) end
  else
    target = Body.get_inverse_rarm(p)
    if target then Body.set_rarm_command(target) end
  end
  
  print('\ncur', unpack(Body.get_rarm_position()) )
  print('\nik', unpack(target) )
  
  -- Return the status
  if target then
    return string.format( 'Updated arm angles via IK' )
  else
    return 'Invalid IK update!'
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
  local text = string.format('%s:\t',arm_name)
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

local function state_msg()
  local msg = '=========\nKeyboard Wizard\n'
  msg = msg..'Current State'

  local larm_cmd = Body.get_larm_command()
  local rarm_cmd = Body.get_rarm_command()
  local lfinger_cmd = vector.slice(Body.get_aux_command(),1,3)
  local rfinger_cmd = vector.slice(Body.get_aux_command(),4,6)

  local larm = Body.get_larm_position()
  local rarm = Body.get_rarm_position()
  local lfinger = vector.slice(Body.get_aux_position(),1,3)
  local rfinger = vector.slice(Body.get_aux_position(),4,6)
  
  local left  = jangle_str('left', larm,lfinger)
  local right = jangle_str('right',rarm,rfinger)
  local left_cmd  = jangle_str('left', larm_cmd,lfinger_cmd)
  local right_cmd = jangle_str('right',rarm_cmd,rfinger_cmd)

  
  local cur = 'Operating on '..current_arm..' '..joint_name()..' in radians'
  local m = 'Control Mode: '..mode_msg[current_mode]
  local pL = Body.get_forward_larm()
  local pR = Body.get_forward_rarm()
  local lik = string.format(
    'Left  IK:\t(%.2f  %.2f  %.2f) (%.2f  %.2f  %.2f)',unpack(pL)
  )
  local rik = string.format(
    'Right IK:\t(%.2f  %.2f  %.2f) (%.2f  %.2f  %.2f)',unpack(pR)
  )
  
  return string.format(
  '======\nKeyboard Wizard State\n%s\n%s\n%s\n%s\n%s\n%s\n\n%s\n%s',
  cur,m,lik,rik,left,right,left_cmd,right_cmd)
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
    current_arm = 'left'
    return switch_msg()
  elseif key_char==']' then
    current_arm = 'right'
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
    if current_mode>#mode_msg then
      current_mode = 1
    end
    return string.format('Switched to %s mode',mode_msg[current_mode])
  end
  
  -- IK Changes with wasd
  local ik_update = nil
  if key_char_lower=='w' then
    return ik_change( delta_ik, 0, 0 )
  elseif key_char_lower=='a' then
    return ik_change( 0, delta_ik, 0 )
  elseif key_char_lower=='s' then
    return ik_change( -delta_ik, 0, 0 )
  elseif key_char_lower=='d' then
    return ik_change( 0, -delta_ik, 0 )
  elseif key_char_lower=='q' then
    return ik_change( 0, 0, delta_ik )
  elseif key_char_lower=='z' then
    return ik_change( 0, 0, -delta_ik )
  end

  -- Default message
  return 'Nothing changes'
  
end

------------
-- Start processing
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
  
  -- Print result of the key press
  os.execute("clear")
  io.write( '\n\n', msg )
  io.write( '\n\n', state_msg() )
  io.flush()
    
end
