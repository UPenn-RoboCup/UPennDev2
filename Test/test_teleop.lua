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
local fromQ = require'Transform'.from_quatp
local toQ = require'Transform'.to_quatp
local movearm = require'movearm'

-- Look up tables for the test.lua script (NOTE: global)
code_lut, char_lut, lower_lut = {}, {}, {}

local narm = 7 -- TODO: Use the config to check...
local selected_arm = 0 -- left to start
--
local LARM_DIRTY = false
local qLtmp = vector.zeros(narm)
local qL0 = vector.zeros(narm)
local function get_larm(refresh)
	if refresh then qLtmp = hcm.get_teleop_larm() end
	return qLtmp, qL0
end
local function set_larm(q, do_now)
	if type(q)=='table' and #q==#qLtmp then
		vector.copy(q, qLtmp)
		LARM_DIRTY = true
	end
	if q==true or do_now==true then
		hcm.set_teleop_larm(qLtmp)
		vector.copy(qLtmp, qL0)
		LARM_DIRTY = false
	end
end
--
local RARM_DIRTY = false
local qRtmp = vector.zeros(narm)
local qR0 = vector.zeros(narm)
local function get_rarm(refresh)
	if refresh then qRtmp = hcm.get_teleop_rarm() end
	return qRtmp, qR0
end
local function set_rarm(q, do_now)
	if type(q)=='table' and #q==#qRtmp then
		vector.copy(q, qRtmp)
		RARM_DIRTY = true
	end
	if q==true or do_now==true then
		hcm.set_teleop_rarm(qRtmp)
		vector.copy(qRtmp, qR0)
		RARM_DIRTY = false
	end
end
--
local TFLARM_DIRTY = false
local tfLtmp = T.eye()
local tfL0 = T.eye()
local function get_tflarm(refresh)
	if refresh then
		tfLtmp = fromQ(hcm.get_teleop_tflarm())
	end
	return tfLtmp, tfL0
end
local function set_tflarm(tf, do_now)
	if type(tf)~='boolean' then
		T.copy(tf, tfLtmp)
		TFLARM_DIRTY = true
	end
	if tf==true or do_now==true then
		hcm.set_teleop_tflarm(toQ(tfLtmp))
		T.copy(tfLtmp, tfL0)
		TFLARM_DIRTY = false
	end
end
--
local TFRARM_DIRTY = false
local tfRtmp = T.eye()
local tfR0 = T.eye()
local function get_tfrarm(refresh)
	if refresh then
		tfRtmp = fromQ(hcm.get_teleop_tfrarm())
	end
	return tfRtmp, tfR0
end
local function set_tfrarm(tf, do_now)
	if type(tf)~='boolean' then
		T.copy(tf, tfRtmp)
		TFRARM_DIRTY = true
	end
	if tf==true or do_now==true then
		hcm.set_teleop_tfrarm(toQ(tfRtmp))
		T.copy(tfRtmp, tfR0)
		TFRARM_DIRTY = false
	end
end
--
local HEAD_DIRTY = false
local qHeadtmp = vector.zeros(2)-- tracking this
local qHead0 = vector.zeros(2) -- last sent
local function get_head(refresh)
	if refresh then qHeadtmp = hcm.get_teleop_head() end
	return qHeadtmp, qHead0
end
local function set_head(q, do_now)
	if type(q)=='table' and #q==#qHeadtmp then
		vector.copy(q, qHeadtmp)
		HEAD_DIRTY = true
	end
	if q==true or do_now==true then
		hcm.set_teleop_head(qHeadtmp)
		vector.copy(qHeadtmp, qHead0)
		HEAD_DIRTY = false
	end
end

-- Immediately write the changes?
local DO_IMMEDIATE = true
char_lut["'"] = function()
  DO_IMMEDIATE = not DO_IMMEDIATE
end

-- Sync the delayed sending
function sync()
	if LARM_DIRTY then set_larm(true) else vector.copy(get_larm(true), qL0) end
	if RARM_DIRTY then set_rarm(true) else vector.copy(get_rarm(true), qR0) end
	if HEAD_DIRTY then set_head(true) else vector.copy(get_head(true), qHead0) end
	if TFLARM_DIRTY then set_tflarm(true) else T.copy(get_tflarm(true), tfL0) end
	if TFRARM_DIRTY then set_tfrarm(true) else T.copy(get_tfrarm(true), tfR0) end
end
char_lut[' '] = sync

-- Enter syncs the data
local body_state
local head_state
local arm_state
local motion_state
local gripper_state
local function sync_fsm()
	body_state = gcm.get_fsm_Body()
	head_state = gcm.get_fsm_Head()
	arm_state = gcm.get_fsm_Arm()
	motion_state = gcm.get_fsm_Motion()
	gripper_state = gcm.get_fsm_Gripper()
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
	head_ch:send'trackleft'
end
char_lut['5'] = function()
  head_ch:send'trackright'
end
char_lut['6'] = function()
	arm_ch:send'teleopraw'
	unix.usleep(1e5)
	vector.copy(get_larm(true), qL0)
	vector.copy(get_rarm(true), qR0)
end
char_lut['7'] = function()
	arm_ch:send'teleop'
	unix.usleep(1e5)
	T.copy(get_tflarm(true), tfL0)
	T.copy(get_tfrarm(true), tfR0)
end
char_lut['8'] = function()
	body_ch:send'stop'
end
char_lut['9'] = function()
  motion_ch:send'hybridwalk'
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

char_lut['g'] = function()
  motion_ch:send'getup'
end

local function apply_pre(d_tr)
	if selected_arm==0 then --left
		local tfLArm = get_tflarm()
		local tfLGoal = d_tr * tfLArm
		set_tflarm(tfLGoal, DO_IMMEDIATE)
	else
		local tfRArm = get_tfrarm()
		local tfRGoal = d_tr * tfRArm
		set_tfrarm(tfRGoal, DO_IMMEDIATE)
	end
end

local function apply_post(d_tr)
	if selected_arm==0 then --left
		local tfLArm = get_tflarm()
		local tfLGoal = tfLArm * d_tr
		set_tflarm(tfLGoal, DO_IMMEDIATE)
	else
		local tfRArm = get_tfrarm()
		local tfRGoal = tfRArm * d_tr
		set_tfrarm(tfRGoal, DO_IMMEDIATE)
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
  local goalBefore = get_head()
  local goalAfter = goalBefore + dHead
  set_head(goalAfter, DO_IMMEDIATE)
end

--[[
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
--]]

-- Add the access to the transforms
setmetatable(lower_lut, {
	__index = function(t, k)
    if (not arm_mode) then
      if head[k] then
				return function() apply_head(head[k]) end
--[[
      elseif k=='k' then
        return function() mcm.set_walk_vel({0,0,0}) end
      else
        return function() apply_walk(walk[k]) end
--]]
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
  local l_indicator = {}
	local r_indicator = {}
	for i=1,narm do
		local indicator = selected_joint == i and i or '#'
		table.insert(l_indicator, selected_arm==0 and indicator or '#')
		table.insert(r_indicator, selected_arm==1 and indicator or '#')
	end
	--
	local qL, qL0 = get_rarm()
	local tfL, tfL0 = get_tflarm()
  local larm_info = table.concat({string.format('\n%s %s\t%s',
    util.color('Left Arm', 'yellow'),
    arm_mode and selected_arm==0 and '*' or ' ',
		table.concat(l_indicator, ' ')),
    'qRobot: '..tostring(qL0*RAD_TO_DEG),
    'qOperator: '..tostring(qL*RAD_TO_DEG),
		'tfRobot: '..T.string6D(tfL0),
    'tfOperator: '..T.string6D(tfL)
  },'\n')
	--
	local qR, qR0 = get_rarm()
	local tfR, tfR0 = get_tfrarm()
  local rarm_info = table.concat({string.format('%s %s\t%s',
    util.color('Right Arm', 'yellow'),
    arm_mode and selected_arm==1 and '*' or ' ',
		table.concat(r_indicator, ' ')),
		'qRobot: '..tostring(qR0*RAD_TO_DEG),
    'qOperator: '..tostring(qR*RAD_TO_DEG),
		'tfRobot: '..T.string6D(tfR0),
    'tfOperator: '..T.string6D(tfR)
  },'\n')
	--
	local qh, qh0 = get_head()
  local head_info = string.format('\n%s %s\n%s\n%s',
    util.color('Head', 'yellow'),
    (not arm_mode) and '*' or '',
		'qRobot: '..tostring(qh0*RAD_TO_DEG),
    'qOperator: '..tostring(qh*RAD_TO_DEG)
  )
	--
  local info = {
    color('== Teleoperation ==', 'magenta'),
		color(DO_IMMEDIATE and 'Immediate Send' or 'Delayed Send', DO_IMMEDIATE and 'red' or 'yellow'),
    'BodyFSM: '..color(body_state, 'green'),
    'ArmFSM: '..color(arm_state, 'green'),
    'HeadFSM: '..color(head_state, 'green'),
    'MotionFSM: '..color(motion_state, 'green'),
    larm_info,
    rarm_info,
    head_info,
    '\n'
  }
  if not IS_WEBOTS then io.write(table.concat(info,'\n')) end
end

-- Initial sync
sync_fsm()
sync()

-- Run the generic keypress library
return dofile(HOME..'/Test/test.lua')
