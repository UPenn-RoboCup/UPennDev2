#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
pcall(dofile,'fiddle.lua')
pcall(dofile, '../fiddle.lua')
local WAS_REQUIRED = ... and type(...)=='string'

local K = require'K_ffi'
local T = require'libTransform'

local narm = #Body.get_larm_position()
local selected_arm = 0 -- left

-- Look up tables
local code_lut, char_lut, lower_lut = {}, {}, {}
-- 
char_lut['1'] = function()
  motion_ch:send'stand'
	arm_ch:send'init'
end
char_lut['2'] = function()
	arm_ch:send'ready'
end
char_lut['3'] = function()
	arm_ch:send'poke'
end
char_lut['0'] = function()
	arm_ch:send'teleop'
end
char_lut[' '] = function()
  -- Debugging
  local qLArm = Body.get_larm_command_position()
	local qRArm = Body.get_larm_command_position()
  print('qLArm',qLArm*RAD_TO_DEG)
	print('qRArm',qRArm*RAD_TO_DEG)
  print()
	--local fk = K.forward_arm(qArm)
  --print(T.tostring(fk))
  --print()
	--[[
  local zyz = T.to_zyz(fk)*RAD_TO_DEG
  print('zyz:',zyz[1],zyz[2],zyz[3])
	print()
	--]]
	-- Show the forward mass model
	--[[
	local com = vector.new( K.forward_com(qArm, {0,0,0}, 0) )
	local com_arm = vector.new( K.forward_com_arm(qArm, {0,0,0}, 0) )
	print('Full COM',com)
	print('Arm  COM',com_arm)
	--]]
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
lower_lut.g = function()
  if selected_arm==0 then
    local qLArm = Body.get_larm_command_position()
		local tr = K.forward_l_arm(qLArm)
		local iqArm = K.inverse_l_arm(tr, qLArm, qLArm[3] - DEG_TO_RAD)
		local itr = K.forward_l_arm(iqArm)
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
  else
    local qRArm = Body.get_rarm_command_position()
		local tr = K.forward_r_arm(qRArm)
		local iqArm = K.inverse_r_arm(tr, qRArm, qRArm[3] - DEG_TO_RAD)
		local itr = K.forward_r_arm(iqArm)
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
  end
end

lower_lut.h = function()
	print('selected_arm', selected_arm)
  if selected_arm==0 then
    local qLArm = Body.get_larm_command_position()
		local tr = K.forward_l_arm(qLArm)
		local iqArm = K.inverse_l_arm(tr, qLArm, qLArm[3] + DEG_TO_RAD)
		local itr = K.forward_l_arm(iqArm)
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
  else
    local qRArm = Body.get_rarm_command_position()
		local tr = K.forward_r_arm(qRArm)
		local iqArm = K.inverse_r_arm(tr, qRArm, qRArm[3] + DEG_TO_RAD)
		local itr = K.forward_r_arm(iqArm)
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
		local fkL = K.forward_l_arm(qLArm)
		local trLGoal = d_tr * fkL
		local iqArm = vector.new(K.inverse_l_arm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
	else
		local qRArm = Body.get_rarm_command_position()
		local fkR = K.forward_r_arm(qRArm)
		local trRGoal = d_tr * fkR
		local iqArm = vector.new(K.inverse_r_arm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
	end
end

local function apply_post(d_tr)
	if selected_arm==0 then --left
		local qLArm = Body.get_larm_command_position()
		local fkL = K.forward_l_arm(qLArm)
		local trLGoal = fkL * d_tr
		local iqArm = vector.new(K.inverse_l_arm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		Body.set_larm_command_position(iqArm)
	else
		local qRArm = Body.get_rarm_command_position()
		local fkR = K.forward_r_arm(qRArm)
		local trRGoal = fkR * d_tr
		local iqArm = vector.new(K.inverse_r_arm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		Body.set_rarm_command_position(iqArm)
	end
end

local function show_status()
  if not IS_WEBOTS then os.execute('clear') end
  print(util.color('Remote Control', 'magenta'))
  print('Motion:', util.color(gcm.get_fsm_Motion(), 'green'))
  
	local qlarm = Body.get_larm_position()
	local qrarm = Body.get_rarm_position()
	
	local fkL = K.forward_l_arm(qlarm)
	local fkR = K.forward_r_arm(qrarm)
	
  local l_indicator = vector.zeros(#qlarm)
  l_indicator[selected_joint] = selected_arm==0 and 1 or 0
  local r_indicator = vector.zeros(#qlarm)
  r_indicator[selected_joint] = selected_arm==1 and 1 or 0

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

local function update(key_code)
  -- Assure that we got a character
  if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
  
  local code_f, char_f, lower_f, pre_tr, post_tr = code_lut[key_code], char_lut[key_char], lower_lut[key_char_lower], pre_arm[key_char], post_arm[key_char]
  --print('Codes', key_code, key_char, key_char_lower)
  -- Precedence
  if type(code_f)=='function' then
    code_f()
  elseif type(char_f)=='function' then
    char_f()
  elseif type(lower_f)=='function' then
    lower_f()
	elseif pre_tr then
		apply_pre(pre_tr)
	elseif post_tr then
		apply_post(post_tr)
  end
	
	show_status()
end

-- Show the initial status
show_status()

if WAS_REQUIRED then return {entry=nil, update=update, exit=nil} end

local getch = require'getch'
local running = true
local key_code
while running do
	key_code = getch.block()
  update(key_code)
end
