#!/usr/bin/env luajit
-- (c) 2014, 2015 Stephen McGill
local IS_REMOTE = false
if type(arg)=='table' then
	for i, a in ipairs(arg) do
		if a:find'r' then IS_REMOTE = true end
	end
end

local helper = (IS_REMOTE and 'riddle' or 'fiddle')..'.lua'
local ok, err = pcall(dofile, helper)
if not ok then
	local ok, err2 = pcall(dofile, '../'..helper)
	assert(ok, tostring(err)..'\n'..tostring(err2))
end

local vector = require'vector'
local T = require'Transform'
local K = require'K_ffi'
local sanitize = K.sanitize

-- Look up tables for the test.lua script (NOTE: global)
code_lut, char_lut, lower_lut = {}, {}, {}

local narm = 7 -- TODO: Use the config to check...
local selected_arm = 0 -- left to start

local DO_IMMEDIATE = true
local LARM_DIRTY, RARM_DIRTY = false, false
local qLtmp, qL0
local function get_larm(refresh)
	if refresh then qLtmp = hcm.get_teleop_larm() end
	return qLtmp
end

local function set_larm(q, do_now)
	if type(q)=='table' and #q==#qLtmp then
		vector.copy(q, qLtmp)
		LARM_DIRTY = true
	end
	if q==true or do_now==true then
		LARM_DIRTY = false
		local curTeleop = hcm.get_teleop_larm()
		if curTeleop~=qL0 then
			--print('TEST_TELEOP | L Outdated...')
			vector.copy(curTeleop, qL0)
			qLtmp = curTeleop
			return
		end
		hcm.set_teleop_larm(qLtmp)
		vector.copy(qLtmp, qL0)
		arm_ch:send'teleop'
	end
end
local qRtmp, qR0
local function get_rarm(refresh)
	if refresh then qRtmp = hcm.get_teleop_rarm() end
	return qRtmp
end

local function set_rarm(q, do_now)
	if type(q)=='table' and #q==#qRtmp then
		vector.copy(q, qRtmp)
		RARM_DIRTY = true
	end
	if q==true or do_now==true then
		LARM_DIRTY = false
		local curTeleop = hcm.get_teleop_rarm()
		if curTeleop~=qR0 then
			--print('TEST_TELEOP | R Outdated...')
			vector.copy(curTeleop, qR0)
			qRtmp = curTeleop
			return
		end
		hcm.set_teleop_rarm(qRtmp)
		vector.copy(qRtmp, qR0)
		arm_ch:send'teleop'
	end
end
-- Immediately write the changes?
char_lut["'"] = function()
  DO_IMMEDIATE = not DO_IMMEDIATE
end

-- Sync the delayed sending
char_lut[' '] = function()
	if LARM_DIRTY then set_larm(true) end
	if RARM_DIRTY then set_rarm(true) end
end

-- Enter syncs the data
local qlarm
local qrarm
local uComp
local body_state
local head_state
local arm_state
local motion_state
local gripper_state
local walk_velocity
local function sync()
	qlarm = Body.get_larm_position()
	qrarm = Body.get_rarm_position()
	uComp = mcm.get_stance_uTorsoComp()
	body_state = gcm.get_fsm_Body()
	head_state = gcm.get_fsm_Head()
	arm_state = gcm.get_fsm_Arm()
	motion_state = gcm.get_fsm_Motion()
	gripper_state = gcm.get_fsm_Gripper()
	walk_velocity = mcm.get_walk_vel()
end

-- Backspace (Win/Linux) / Delete (OSX)
local USE_COMPENSATION
code_lut[127] = function()
	-- Disable the compensation
	USE_COMPENSATION = hcm.get_teleop_compensation()
	----[[
	USE_COMPENSATION = USE_COMPENSATION + 1
	USE_COMPENSATION = USE_COMPENSATION>2 and 0 or USE_COMPENSATION
	--]]
	USE_COMPENSATION = USE_COMPENSATION==1 and 2 or 1
	hcm.set_teleop_compensation(USE_COMPENSATION)
	arm_ch:send'teleop'
end

-- Switch to head teleop
local arm_mode = true
char_lut['`'] = function()
  arm_mode = not arm_mode
end

-- State Machine events
char_lut['1'] = function()
  body_ch:send'init'
end
char_lut['2'] = function()
	arm_ch:send'ready'
end
char_lut['3'] = function()
	head_ch:send'teleop'
end
char_lut['4'] = function()
	head_ch:send'teleop'
	arm_ch:send'teleop'
end
char_lut['5'] = function()
  body_ch:send'approach'
end
char_lut['6'] = function()
	arm_ch:send'dean'
	gripper_ch:send'dean'
	--head_ch:send'trackhand'
  --arm_ch:send'poke'
end

char_lut['8'] = function()
	body_ch:send'stop'
end
char_lut['9'] = function()
  motion_ch:send'hybridwalk'
end

lower_lut['r'] = function()
  if selected_arm==0 then
		local options = hcm.get_teleop_loptions()
		options[1] = math.max(options[1] - DEG_TO_RAD, 0)
		hcm.set_teleop_loptions(options)
		arm_ch:send'teleop'
		--[[
		local qLArm = get_larm()
    --print('Pre',qLArm*RAD_TO_DEG)
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] - DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		set_larm(iqArm, DO_IMMEDIATE)
		--]]
  else
		local options = hcm.get_teleop_roptions()
		options[1] = math.min(options[1] - DEG_TO_RAD, 0)
		hcm.set_teleop_roptions(options)
		arm_ch:send'teleop'
		--[[
    local qRArm = get_rarm()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] - DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		set_rarm(iqArm, DO_IMMEDIATE)
		--]]
  end
end

lower_lut['t'] = function()
  if selected_arm==0 then
		local options = hcm.get_teleop_loptions()
		options[1] = math.min(options[1] + DEG_TO_RAD, 90*DEG_TO_RAD)
		hcm.set_teleop_loptions(options)
		arm_ch:send'teleop'
		--[[
    local qLArm = get_larm()
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] + DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		set_larm(iqArm, DO_IMMEDIATE)
		--]]
  else
		local options = hcm.get_teleop_roptions()
		options[1] = math.max(options[1] + DEG_TO_RAD, -90*DEG_TO_RAD)
		hcm.set_teleop_roptions(options)
		arm_ch:send'teleop'
		--[[
    local qRArm = get_rarm()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] + DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		set_rarm(iqArm, DO_IMMEDIATE)
		--]]
  end
end

--
code_lut[92] = function()
  -- Backslash
  selected_arm = 1 - selected_arm
end
local selected_joint = 1
char_lut[']'] = function()
  selected_joint = selected_joint + 1
  selected_joint = math.max(1, math.min(selected_joint, narm))
end
char_lut['['] = function()
  selected_joint = selected_joint - 1
  selected_joint = math.max(1, math.min(selected_joint, narm))
end

char_lut['='] = function()
  if selected_arm==0 then
    local pos = get_larm()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		set_larm(pos, DO_IMMEDIATE)
  else
    local pos = get_rarm()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		set_rarm(pos, DO_IMMEDIATE)
  end
end
char_lut['-'] = function()
  if selected_arm==0 then
    local pos = get_larm()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		set_larm(pos, DO_IMMEDIATE)
  else
    local pos = get_rarm()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		set_rarm(pos, DO_IMMEDIATE)
  end
end

--[[
local zyz = T.to_zyz(desired_tr)
print('des zyz:',zyz[1],zyz[2],zyz[3])
--]]
local function apply_pre(d_tr)
	if selected_arm==0 then --left
		local qLArm = get_larm()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = d_tr * fkL
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		set_larm(iqArm, DO_IMMEDIATE)
	else
		local qRArm = get_rarm()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = d_tr * fkR
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		set_rarm(iqArm, DO_IMMEDIATE)
	end
end

local function apply_post(d_tr)
	if selected_arm==0 then --left
		local qLArm = get_larm()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = fkL * d_tr
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		set_larm(iqArm, DO_IMMEDIATE)
	else
		local qRArm = get_rarm()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = fkR * d_tr
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		set_rarm(iqArm, DO_IMMEDIATE)
	end
end

-- Translate the end effector
local ds = 0.01
local dr = 3 * DEG_TO_RAD
local pre_arm = {
  u = T.trans(0,0,ds),
  m = T.trans(0,0,-ds),
  i = T.trans(ds,0,0),
  [','] = T.trans(-ds,0,0),
  j = T.trans(0,ds,0),
  l = T.trans(0,-ds,0),
	y = T.rotY(dr),
  n = T.rotY(-dr),
}

-- Rotate (locally) the end effector
local post_arm = {
  e = T.rotY(dr),
  c = T.rotY(-dr),
  a = T.rotZ(dr),
  d = T.rotZ(-dr),
	b = T.rotX(dr),
  v = T.rotX(-dr),
  ["q"] = T.trans(0,0,ds),
  ['z'] = T.trans(0,0,-ds),
  ["w"] = T.trans(ds,0,0),
  ['x'] = T.trans(-ds,0,0),
}

local dHead = 5*DEG_TO_RAD
local head = {
  w = dHead * vector.new{0,-1},
  a = dHead * vector.new{1, 0},
  s = dHead * vector.new{0,1},
  d = dHead * vector.new{-1, 0},
}
local function apply_head(dHead)
  if not dHead then return end
  local goalBefore = hcm.get_teleop_head()
  local goalAfter = goalBefore + dHead
  hcm.set_teleop_head(goalAfter)
end

local dWalk = 0.05
local daWalk = 5*DEG_TO_RAD
local walk = {
  i = dWalk * vector.new{1, 0, 0},
  [','] = dWalk * vector.new{-1, 0, 0},
  --
  j = dWalk * vector.new{0, 0, 1},
  l = dWalk * vector.new{0, 0, -1},
  --
  h = dWalk * vector.new{0, 1, 0},
  [';'] = dWalk * vector.new{0, -1, 0},
}
local function apply_walk(dWalk)
  if not dWalk then return end
  local goalBefore = mcm.get_walk_vel()
  local goalAfter = goalBefore + dWalk
  mcm.set_walk_vel(goalAfter)
end

-- Add the access to the transforms
setmetatable(lower_lut, {
	__index = function(t, k)
    if (not arm_mode) then
      if head[k] then
				return function() apply_head(head[k]) end
      elseif k=='k' then
        return function() mcm.set_walk_vel({0,0,0}) end
      else
        return function() apply_walk(walk[k]) end
      end
    elseif pre_arm[k] then
			return function() apply_pre(pre_arm[k]) end
		elseif post_arm[k] then
			return function() apply_post(post_arm[k]) end
		end
	end
})

-- Global status to show (NOTE: global)
local color = require'util'.color
function show_status()
	
local qrarm = Body.get_rarm_command_position()
	local fkL = K.forward_larm(qlarm)
	local fkR = K.forward_rarm(qrarm)
	local rTr6 = T.position6D(fkR)
	local fkR2 = {rTr6[1]+uComp[1], rTr6[2]+uComp[2], rTr6[3], rTr6[4]*RAD_TO_DEG, rTr6[5]*RAD_TO_DEG, rTr6[6]*RAD_TO_DEG}

  local l_indicator = vector.zeros(#qlarm)
  l_indicator[selected_joint] = selected_arm==0 and 1 or 0
  local r_indicator = vector.zeros(#qlarm)
  r_indicator[selected_joint] = selected_arm==1 and 1 or 0
	--
  local larm_info = string.format('\n%s %s %s\n%s\n%s\n%s',
    util.color('Left Arm', 'yellow'),
    arm_mode and selected_arm==0 and '*' or '',
		l_indicator,
    'q: '..tostring(qlarm*RAD_TO_DEG),
		'tr: '..tostring(vector.new(T.position6D(fkL))),
		'teleop: '..tostring(qLtmp*RAD_TO_DEG)
  )

  local rarm_info = string.format('\n%s %s %s\n%s\n%s\n%s',
    util.color('Right Arm', 'yellow'),
    arm_mode and selected_arm==1 and '*' or '',
		r_indicator,
    'q: '..tostring(qrarm*RAD_TO_DEG),
    string.format('tr: %.2f, %.2f, %.2f | %.2f, %.2f, %.2f', unpack(fkR2)),
    'teleop: '..tostring(qRtmp*RAD_TO_DEG)
  )
  local head_info = string.format('\n%s %s\n%s',
    util.color('Head', 'yellow'),
    (not arm_mode) and '*' or '',
    'q: '..tostring(Body.get_head_position()*RAD_TO_DEG)
  )
  local walk_info = string.format('\n%s %s\n%s',
    util.color('Walk', 'yellow'),
    (not arm_mode) and '*' or '',
    'Velocity: '..tostring(walk_velocity)
  )
  local info = {
    color('== Teleoperation ==', 'magenta'),
		'1: init, 2: head teleop, 3: armReady, 4: armTeleop, 5: headTrack, 6: poke',
		color(DO_IMMEDIATE and 'Immediate Send' or 'Delayed Send', DO_IMMEDIATE and 'red' or 'yellow'),
		'Compensation: '..tostring(USE_COMPENSATION),
    'BodyFSM: '..color(body_state, 'green'),
    'ArmFSM: '..color(arm_state, 'green'),
    'HeadFSM: '..color(head_state, 'green'),
    'MotionFSM: '..color(motion_state, 'green'),
    larm_info,
    rarm_info,
    head_info,
    walk_info,
    '\n'
  }
  if not IS_WEBOTS then io.write(table.concat(info,'\n')) end
end

-- Initial sync
sync()
-- Set initial arms in tmp and 0
qL0 = vector.copy(get_larm(true))
qR0 = vector.copy(get_rarm(true))

-- Run the generic keypress library
return dofile(HOME..'/Test/test.lua')
