-----------------------------------------------------------------
-- Keyboard Wizard
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

--local is_debug = true

-- Libraries
local unix  = require'unix'
local getch = require'getch'
local mp    = require'msgpack'
local util  = require'util'
local Body  = require'Body'
local vector = require'vector'
local simple_ipc = require'simple_ipc'
local rpc_ch = simple_ipc.new_requester(Config.net.reliable_rpc)

local current_joint = 1
local current_arm = 'larm'
local current_grip = 'larm'
-- Modes: 0: direct, 1: ik
local current_mode = 1
local mode_msg = {
  'direct',
  'inverse kinematics'
}
local n_modes = #mode_msg
-- Arm with three fingers
local nArm = #Body.parts['LArm']
local max_joint = nArm+#Body.parts['LGrip']

-- Change in radians for each +/-
local DEG_TO_RAD = math.pi/180
local delta_joint = 1 * DEG_TO_RAD

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
		if current_joint<=nArm then
			jName = Body.jointNames[Body.indexLArm+current_joint-1]
		else
			jName = 'finger '..current_joint-nArm
		end
	elseif current_arm=='rarm' then
		if current_joint<=nArm then
			jName = Body.jointNames[Body.indexRArm+current_joint-1]
		else
			jName = 'finger '..current_joint-nArm
		end
	end
	return jName
end

-- Print Message helpers
local switch_msg = function()
  local sw = string.format('Switched to %s %s.', 
  current_arm, joint_name() )
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
    text = text..' '
    if i==current_joint then
      if current_arm==arm_name:lower() then
        text = text..'*'
      else
        text = text..' '
      end
    end
    text = text..string.format( '%6.3f', v )
  end
  for i,v in ipairs(finger_angles) do
    text = text..' '
    if i+nArm==current_joint then
      if current_arm==arm_name:lower() then
        text = text..'*'
      else
        text = text..' '
      end
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
  local msg = util.color('\nKeyboard Wizard\n','blue')
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
  
  msg = msg..'\nLidar pos:\t'..string.format('%g',Body.get_lidar_position(1))
  
  
  -- Return the message
  return msg
end

local delta_pos = .01 -- meters
local delta_ang = .5*Body.DEG_TO_RAD -- degrees
local char_to_ik = {
  -- Positions
  ['w'] = delta_pos*vector.new({1,0,0,  0,0,0}),
  ['s'] = delta_pos*vector.new({-1,0,0, 0,0,0}),
  ['a'] = delta_pos*vector.new({0,1,0,  0,0,0}),
  ['d'] = delta_pos*vector.new({0,-1,0, 0,0,0}),
  ['q'] = delta_pos*vector.new({0,0,1,  0,0,0}),
  ['z'] = delta_pos*vector.new({0,0,-1, 0,0,0}),
  -- Angles
  ['i'] = delta_ang*vector.new({0,0,0, 0,1,0 }),
  ['k'] = delta_ang*vector.new({0,0,0, 0,-1,0}),
  ['j'] = delta_ang*vector.new({0,0,0, 1,0,0 }),
  ['l'] = delta_ang*vector.new({0,0,0, -1,0,0}),
  ['u'] = delta_ang*vector.new({0,0,0, 0,0,1 }),
  ['m'] = delta_ang*vector.new({0,0,0, 0,0,-1}),
}

local function send_command(cmd)
  -- Default case is to send the command and receive a reply
  local ret   = rpc_ch:send(mp.pack(cmd))
  local reply = rpc_ch:receive()
  return mp.unpack(reply)
end

-- Character processing
local function process_character(key_code,key_char,key_char_lower)

  -- Number keys switch the joint on the arm chain
  -- Number 0 zeros the joint
  local switch_joint = tonumber(key_char)  
  if switch_joint and switch_joint~=0 then
    -- Check that the joint number is in range
    if switch_joint<1 or switch_joint>max_joint then
      return 'Joint '..switch_joint..' out of range!'
    end
    -- Switch to that joint
    current_joint = switch_joint
    print( switch_msg() )
    --print(util.color('Switched to','yellow'),current_arm,current_joint)
    return
  end
  
  -- Bracket keys switch arms
  if key_char=='[' then
    current_arm = 'larm'
    current_grip = 'lgrip'
    print(util.color('Switched to','yellow'),current_arm)
    return
  elseif key_char==']' then
    current_arm = 'rarm'
    current_grip = 'rgrip'
    print(util.color('Switched to','yellow'),current_arm)
    return
  end
  
  -- Switch control modes
  if key_char=='`' then
    current_mode = current_mode + 1
    if current_mode>n_modes then current_mode = 1 end
    local cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'joints'
    cmd.key = 'teleop'
    cmd.val = current_mode
    send_command(cmd)
    print(util.color('Switched to','yellow'),mode_msg[current_mode])
  end
  
  -- +/- Increases and decreases
  local delta_joint = 1*math.pi/180
  local delta_arm_joints = vector.zeros(nArm)
  local is_delta_joint = false
  if key_char=='-' then
    delta_arm_joints[current_joint] = -delta_joint
    is_delta_joint = true
  elseif key_char=='=' then -- +/= are the same key
    delta_arm_joints[current_joint] = delta_joint
    is_delta_joint = true
  elseif key_char=='0' then
    print(util.color('Zeroed','yellow'),'Not implemented yet')
  end
  if is_delta_joint then
    local cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'joints'
    cmd.key = 'q'..current_arm
    cmd.delta = delta_arm_joints
    send_command(cmd)
    print(current_arm..util.color(' Change joints','yellow'),delta_arm_joints)
    return
  end
  
  -- IK Changes with wasd/ijkl
  local ik_change = char_to_ik[key_char_lower]
  if ik_change then
    local cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'joints'
    cmd.key = 'p'..current_arm
    cmd.delta = ik_change
    send_command(cmd)
    print( util.color('Move '..current_arm,'yellow'), ik_change )
    return
  end

  -- Default message
  return 'Nothing changes'
  
end

------------
-- Start processing
os.execute("clear")
io.flush()
print('Keyboard wizard for arm teleop')
local t0 = unix.time()
while true do
  
  -- Grab the keyboard character
  local key_code = getch.block()
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)
  --print(key_code,key_char,key_char_lower)
  
  -- Process the character
  local msg = process_character(key_code,key_char,key_char_lower)
  
  -- Measure the timing
  local t = unix.time()
  local t_diff = t-t0
  t0 = t
  local fps = 1/t_diff
    
end