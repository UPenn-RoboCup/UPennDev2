#!/usr/bin/env luajit
-- (c) 2014 Stephen McGill
pcall(dofile,'fiddle.lua')
pcall(dofile, '../fiddle.lua')

local K = require'K_ffi'
local T = require'Transform'
local narm = #Body.get_larm_position()
local selected_arm = 0 -- left to start

-- Look up tables for the test.lua script (NOTE: global)
code_lut, char_lut, lower_lut = {}, {}, {}

-- State Machine events
char_lut['1'] = function()
  body_ch:send'init'
end
char_lut['2'] = function()
	arm_ch:send'ready'
	head_ch:send'trackhand'
end
char_lut['3'] = function()
	arm_ch:send'poke'
end
lower_lut['4'] = function()
  motion_ch:send'lean'
end
lower_lut['5'] = function()
  motion_ch:send'stepup'
end

-- Sanitize to avoid trouble with wrist yaw
local fabs = math.abs
local function sanitize(iqArm, cur_qArm)
	local diff, mod_diff
	for i, v in ipairs(cur_qArm) do
		diff = iqArm[i] - v
		mod_diff = util.mod_angle(diff)
		if fabs(diff) > fabs(mod_diff) then iqArm[i] = v + mod_diff end
	end
end
lower_lut['g'] = function()
  if selected_arm==0 then
    local qLArm = Body.get_larm_command_position()
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] - DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
  else
    local qRArm = Body.get_rarm_command_position()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] - DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
  end
end

lower_lut['h'] = function()
  if selected_arm==0 then
    local qLArm = Body.get_larm_command_position()
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] + DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
  else
    local qRArm = Body.get_rarm_command_position()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] + DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
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
    local pos = Body.get_larm_command_position()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		Body.set_larm_command_position(pos)
  else
    local pos = Body.get_rarm_command_position()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		Body.set_rarm_command_position(pos)
  end
end
char_lut['-'] = function()
  if selected_arm==0 then
    local pos = Body.get_larm_command_position()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		Body.set_larm_command_position(pos)
  else
    local pos = Body.get_rarm_command_position()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		Body.set_rarm_command_position(pos)
  end
end

--[[
local zyz = T.to_zyz(desired_tr)
print('des zyz:',zyz[1],zyz[2],zyz[3])
--]]
local function apply_pre(d_tr)
	if selected_arm==0 then --left
		local qLArm = Body.get_larm_command_position()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = d_tr * fkL
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
	else
		local qRArm = Body.get_rarm_command_position()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = d_tr * fkR
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
	end
end

local function apply_post(d_tr)
	if selected_arm==0 then --left
		local qLArm = Body.get_larm_command_position()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = fkL * d_tr
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
	else
		local qRArm = Body.get_rarm_command_position()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = fkR * d_tr
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
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
  ['e'] = T.rotY(dr),
  ['c'] = T.rotY(-dr),
  ["a"] = T.rotZ(dr),
  ['d'] = T.rotZ(-dr),
  ["q"] = T.trans(0,0,ds),
  ['z'] = T.trans(0,0,-ds),
  ["w"] = T.trans(ds,0,0),
  ['x'] = T.trans(-ds,0,0),
}

-- Add the access to the transforms
local mt_tr = {
	__index = function(t, k)
		if pre_arm[k] then
			apply_pre(pre_arm[k])
			return Body.get_time
		elseif post_arm[k] then
			apply_post(post_arm[k])
			return Body.get_time
		end
	end
}
setmetatable(char_lut, mt_tr)

-- Global status to show (NOTE: global)
function show_status()
	local qlarm = Body.get_larm_position()
	local qrarm = Body.get_rarm_position()
	local fkL = K.forward_larm(qlarm)
	local fkR = K.forward_rarm(qrarm)
  local l_indicator = vector.zeros(#qlarm)
  l_indicator[selected_joint] = selected_arm==0 and 1 or 0
  local r_indicator = vector.zeros(#qlarm)
  r_indicator[selected_joint] = selected_arm==1 and 1 or 0
	--
  print(util.color('Remote Control', 'magenta'))
  print('Motion:', util.color(gcm.get_fsm_Motion(), 'green'))
  print(string.format('%s %s\n%s\n%s\n%s',
    util.color('Left Arm Position', 'yellow'),
    selected_arm==0 and '*' or '',
		l_indicator,
    'q: '..tostring(qlarm*RAD_TO_DEG),
		'Pos6d: '..tostring(vector.new(T.position6D(fkL)))
  ))
  print(string.format('%s %s\n%s\n%s\n%s',
    util.color('Right Arm Position', 'yellow'),
    selected_arm==1 and '*' or '',
		r_indicator,
    'q: '..tostring(qrarm*RAD_TO_DEG),
    'Pos6d: '..tostring(vector.new(T.position6D(fkR)))
  ))
end

-- Run the generic keypress library
return dofile(HOME..'/Test/test.lua')
