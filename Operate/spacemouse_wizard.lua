-----------------------------------------------------------------
-- Spacemouse Wizard
-- Listens to spacemouse input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

--local is_debug = true

-- Libraries
local unix = require'unix'
local mp = require'msgpack'
local spacemouse = require 'spacemouse'
local colors = require'colors'
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
      return Body.get_larm_command_position(current_joint)
    else
      -- finger
      return Body.get_lgrip_command_position(current_joint-6)
    end
  else
    if current_joint<7 then
      return Body.get_rarm_command_position(current_joint)
    else
      -- finger
      return Body.get_rgrip_command_position(current_joint-6)
    end
  end
end

local function set_joint(val)
  if current_arm=='left' then
    if current_joint<7 then
      Body.set_larm_command_position(val,current_joint)
    else
      -- finger
      Body.set_lgrip_command_position(val,current_joint-6)
    end
  else
    -- Right
    if current_joint<7 then
      Body.set_rarm_command_position(val,current_joint)
    else
      -- finger
      Body.set_rgrip_command_position(val,current_joint-6)
    end
  end
end

-- Change the joints via IK
local function ik_change( dTrans )
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
  p = p + dTrans 
  
  if current_arm=='left' then
    target = Body.get_inverse_larm(p)
    if target then Body.set_larm_command_position(target) end
  else
    target = Body.get_inverse_rarm(p)
    if target then Body.set_rarm_command_position(target) end
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
  local msg = colors.wrap('\nSpacemouse Wizard\n','blue')
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
    current_joint = current_joint-1
    if current_joint<1 then current_joint = max_joint end
    return''
    --return switch_msg()
  elseif btn==2 then
    current_joint = current_joint+1
    if current_joint>max_joint then current_joint = 1 end
    return''
    --return switch_msg()
  elseif btn==0 then
    return''
  elseif btn==3 then
    -- Switch to that joint
    if current_arm == 'left' then
      current_arm = 'right'
    elseif current_arm == 'right' then
      current_arm = 'left'
    end
    return''
  end
  return'bad button!'
end

local trans_scale = 0.01 / 350
local function process_translate( data )
  local delta_ik = trans_scale * vector.new({data.x,data.y,data.z,0,0,0})
  if vector.norm(delta_ik)<0.003 then return nil end
  return ik_change( delta_ik )
end

local rot_scale = 1 * (math.pi/180)/350
local function process_rotate(data)
  local delta_ik = rot_scale * vector.new({0,0,0,data.wx,data.wy,data.wz})
  if vector.norm(delta_ik)<0.003 then return nil end
  return ik_change( delta_ik )
end

local function direct_rotate(dat)
  local current = get_joint()
  local new = current - rot_scale * data.wz
  set_joint(new)
  return''
end

------------
-- Start processing
os.execute("clear")
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
    print( '\n\n', state_msg() )
    print(colors.wrap(msg,'yellow'))
    msg = 'Unknown'
  end
  unix.usleep( update_interval )
end
