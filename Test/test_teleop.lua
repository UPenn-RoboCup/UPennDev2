#!/usr/bin/env luajit
-- (c) 2014 Stephen McGill
pcall(dofile,'fiddle.lua')
pcall(dofile, '../fiddle.lua')

local T = require'Transform'
local K = require'K_ffi'
local sanitize = K.sanitize

local narm = #Body.get_larm_position()
local selected_arm = 0 -- left to start

-- Look up tables for the test.lua script (NOTE: global)
code_lut, char_lut, lower_lut = {}, {}, {}

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
  head_ch:send'teleop'
end
--[[
char_lut['3'] = function()
  arm_ch:send'grab'
  head_ch:send'trackhand'
end
--]]
char_lut['8'] = function()
  head_ch:send'teleop'
  motion_ch:send'stand'
	if mcm.get_walk_ismoving()>0 then
		mcm.set_walk_stoprequest(1)
	end
end
char_lut['9'] = function()
  head_ch:send'teleop'
  motion_ch:send'hybridwalk'
end

char_lut['r'] = function()
  if selected_arm==0 then
    local qLArm = hcm.get_teleop_larm()
    --print('Pre',qLArm*RAD_TO_DEG)
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] - DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		hcm.set_teleop_larm(iqArm)
  else
    local qRArm = hcm.get_teleop_rarm()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] - DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		hcm.set_teleop_rarm(iqArm)
  end
end

char_lut['t'] = function()
  if selected_arm==0 then
    local qLArm = hcm.get_teleop_larm()
		local tr = K.forward_larm(qLArm)
		local iqArm = K.inverse_larm(tr, qLArm, qLArm[3] + DEG_TO_RAD)
		local itr = K.forward_larm(iqArm)
		sanitize(iqArm, qLArm)
		hcm.set_teleop_larm(iqArm)
  else
    local qRArm = hcm.get_teleop_rarm()
		local tr = K.forward_rarm(qRArm)
		local iqArm = K.inverse_rarm(tr, qRArm, qRArm[3] + DEG_TO_RAD)
		local itr = K.forward_rarm(iqArm)
		sanitize(iqArm, qRArm)
		hcm.set_teleop_rarm(iqArm)
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
    local pos = hcm.get_teleop_larm()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		hcm.set_teleop_larm(pos)
  else
    local pos = hcm.get_teleop_rarm()
    local q0 = pos[selected_joint]
    q0 = q0 + 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		hcm.set_teleop_rarm(pos)
  end
end
char_lut['-'] = function()
  if selected_arm==0 then
    local pos = hcm.get_teleop_larm()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		hcm.set_teleop_larm(pos)
  else
    local pos = hcm.get_teleop_rarm()
    local q0 = pos[selected_joint]
    q0 = q0 - 5 * DEG_TO_RAD
    pos[selected_joint] = q0
		hcm.set_teleop_rarm(pos)
  end
end

--[[
local zyz = T.to_zyz(desired_tr)
print('des zyz:',zyz[1],zyz[2],zyz[3])
--]]
local function apply_pre(d_tr)
	if selected_arm==0 then --left
		local qLArm = hcm.get_teleop_larm()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = d_tr * fkL
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		hcm.set_teleop_larm(iqArm)
	else
		local qRArm = hcm.get_teleop_rarm()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = d_tr * fkR
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		hcm.set_teleop_rarm(iqArm)
	end
end

local function apply_post(d_tr)
	if selected_arm==0 then --left
		local qLArm = hcm.get_teleop_larm()
		local fkL = K.forward_larm(qLArm)
		local trLGoal = fkL * d_tr
		local iqArm = vector.new(K.inverse_larm(trLGoal, qLArm))
		sanitize(iqArm, qLArm)
		hcm.set_teleop_larm(iqArm)
	else
		local qRArm = hcm.get_teleop_rarm()
		local fkR = K.forward_rarm(qRArm)
		local trRGoal = fkR * d_tr
		local iqArm = vector.new(K.inverse_rarm(trRGoal, qRArm))
		sanitize(iqArm, qRArm)
		hcm.set_teleop_rarm(iqArm)
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
setmetatable(char_lut, {
	__index = function(t, k)
    if (not arm_mode) then
      apply_head(head[k])
      if k=='k' then
        mcm.set_walk_vel({0,0,0})
      else
        apply_walk(walk[k])
      end
    elseif pre_arm[k] then
			apply_pre(pre_arm[k])
		elseif post_arm[k] then
			apply_post(post_arm[k])
		end
    return Body.get_time
	end
})

-- Global status to show (NOTE: global)
local color = require'util'.color
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
  local larm_info = string.format('\n%s %s %s\n%s\n%s',
    util.color('Left Arm', 'yellow'),
    arm_mode and selected_arm==0 and '*' or '',
		l_indicator,
    'q: '..tostring(qlarm*RAD_TO_DEG),
		'Pos6d: '..tostring(vector.new(T.position6D(fkL)))
  )
  local rarm_info = string.format('\n%s %s %s\n%s\n%s',
    util.color('Right Arm', 'yellow'),
    arm_mode and selected_arm==1 and '*' or '',
		r_indicator,
    'q: '..tostring(qrarm*RAD_TO_DEG),
    'Pos6d: '..tostring(vector.new(T.position6D(fkR)))
  )
  local head_info = string.format('\n%s %s\n%s',
    util.color('Head', 'yellow'),
    (not arm_mode) and '*' or '',
    'q: '..tostring(Body.get_head_position()*RAD_TO_DEG)
  )
  local walk_info = string.format('\n%s %s\n%s\n%s',
    util.color('Walk', 'yellow'),
    (not arm_mode) and '*' or '',
    'Velocity: '..tostring(mcm.get_walk_vel()),
    'Odometry:'..tostring(mcm.get_status_odometry())
  )
  local info = {
    color('== Teleoperation ==', 'magenta'),
    'BodyFSM: '..color(gcm.get_fsm_Body(), 'green'),
    'ArmFSM: '..color(gcm.get_fsm_Arm(), 'green'),
--    'HeadFSM: '..color(gcm.get_fsm_Head(), 'green'),
--    'MotionFSM: '..color(gcm.get_fsm_Motion(), 'green'),
    larm_info,
    rarm_info,
    head_info,
    walk_info,
    '\n'
  }
  io.write(table.concat(info,'\n'))
end

-- Run the generic keypress library
return dofile(HOME..'/Test/test.lua')
