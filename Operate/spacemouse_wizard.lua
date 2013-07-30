-----------------------------------------------------------------
-- Keyboard Wizard
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

--local is_debug = true

-- Libraries
local unix = require'unix'
local mp = require'msgpack'
local spacemouse = require 'spacemouse'
--sm = spacemouse.init(0x046d, 0xc62b) -- pro
local sm = spacemouse.init(0x046d, 0xc626) -- regular
-- Update every 10ms
local update_interval = 0.010 * 1e6

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
  
  -- Increment the position
  p[1] = p[1] + dx
  p[2] = p[2] + dy
  p[3] = p[3] + dz
  
  if current_arm=='left' then
    target = Body.get_inverse_larm(p)
    if target then Body.set_larm_command(target) end
  else
    target = Body.get_inverse_rarm(p)
    if target then Body.set_rarm_command(target) end
  end
  
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
  local larm_cmd = Body.get_larm_command()
  local rarm_cmd = Body.get_rarm_command()
  local lfinger_cmd = vector.slice(Body.get_aux_command(),1,3)
  local rfinger_cmd = vector.slice(Body.get_aux_command(),4,6)

  -- Position
  local larm = Body.get_larm_position()
  local rarm = Body.get_rarm_position()
  local lfinger = vector.slice(Body.get_aux_position(),1,3)
  local rfinger = vector.slice(Body.get_aux_position(),4,6)
  
  -- Load
  local larm_load = Body.get_larm_load()
  local rarm_load = Body.get_rarm_load()
  local lfinger_load = vector.slice(Body.get_aux_load(),1,3)
  local rfinger_load = vector.slice(Body.get_aux_load(),4,6)
  
  -- Torque Enable
  local larm_en = Body.get_larm_torque_enable()
  local rarm_en = Body.get_rarm_torque_enable()
  local lfinger_en = vector.slice(Body.get_aux_torque_enable(),1,3)
  local rfinger_en = vector.slice(Body.get_aux_torque_enable(),4,6)
  
  -- Inverse Kinematics
  local pL = Body.get_forward_larm()
  local pR = Body.get_forward_rarm()
  
  -- Make the message
  local msg = '=========\nKeyboard Wizard\n'
  msg = msg..'Current State\n'
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
  msg = msg..'\n\nLeft  pos\t'..jangle_str('left', larm,lfinger)
  msg = msg..'\nRight pos\t'..jangle_str('right',rarm,rfinger)
  msg = msg..'\n\nLeft  cmd\t'..jangle_str('left', larm_cmd,lfinger_cmd)
  msg = msg..'\nRight cmd\t'..jangle_str('right',rarm_cmd,rfinger_cmd)
  msg = msg..'\n\nLeft  load\t'..jangle_str('left', larm_load,lfinger_load)
  msg = msg..'\nRight load\t'..jangle_str('right',rarm_load,rfinger_load)
  msg = msg..'\n\nLeft  enable\t'..jangle_str('left', larm_en,lfinger_en)
  msg = msg..'\nRight enable\t'..jangle_str('right',rarm_en,rfinger_en)
  
  -- Return the message
  return msg
end

local function process_button(btn)
  -- Bracket keys switch arms
  if btn==1 then
    current_arm = 'left'
    return''
    --return switch_msg()
  elseif btn==2 then
    current_arm = 'right'
    return''
    --return switch_msg()
  elseif btn==0 then
    return''
  elseif btn==3 then
    -- Switch to that joint
    current_joint = current_joint+1
    if current_joint>max_joint then current_joint = 1 end
    return''
  end
  return'bad button!'
end

local trans_scale = 0.01 / 350
local function process_translate( data )
  local delta_ik = trans_scale * vector.new({data.x,data.y,data.z})
  if vector.norm(delta_ik)<0.003 then return nil end
  return ik_change( unpack(delta_ik) )
end

local rot_scale = 1 * (math.pi/180)/350
local function process_rotate(data)
  local current = get_joint()
  local new = current - rot_scale * data.wz
  set_joint(new)
  return''
end

------------
-- Start processing
io.write( '\n\n',state_msg() )
io.flush()
local msg = 'Unknown'
while true do
  local t = unix.time()
  local evt, data = sm:get()
  if evt=='button' then msg=process_button(data) end
  if evt=='rotate' then process_rotate(data) end
  if evt=='translate' then process_translate(data) end
  -- Print result of the key press
  if evt then
    os.execute("clear")
    io.write( '\n\n', state_msg() )
    io.write( '\n\n', msg )
    msg = 'Unknown'
    io.flush()
  end
  unix.usleep( update_interval )
end