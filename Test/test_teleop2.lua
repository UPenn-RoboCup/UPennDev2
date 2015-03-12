#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

local t_last = Body.get_time()
local tDelay = 0.005*1E6



local angle_increment = 5*math.pi/180



DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 1/DEG_TO_RAD


local override_target=vector.new({0,0,0,  0,0,0,0})

local movement_target=vector.new({0,0,0})

local multiplier = 1

local char_to_override = {
  ['i'] = vector.new({0.04, 0, 0,   0,0,0}),
  [','] = vector.new({-.04, 0, 0,   0,0,0}),
  ['j'] = vector.new({0, 0.04, 0,   0,0,0}),
  ['l'] = vector.new({0, -.04, 0,   0,0,0}),
  ['u'] = vector.new({0, 0, 0.04,  0,0,0}),
  ['m'] = vector.new({0,0, -.04,   0,0,0}),
  
  --Yaw
  ['['] = vector.new({0,0,0,     0,0,1}) *5*math.pi/180,
  [']'] = vector.new({0,0,0,    0,0,-1}) *5*math.pi/180,

  --Pitch
  ['y'] = vector.new({0,0,0,     0,-1, 0}) *5*math.pi/180,
  ['n'] = vector.new({0,0,0,     0,1,  0}) *5*math.pi/180,

  --Roll
  ['o'] = vector.new({0,0,0,     -1,0, 0}) *5*math.pi/180,
  ['.'] = vector.new({0,0,0,     1,0,  0}) *5*math.pi/180,
}

local char_to_movetarget = {
	['w'] = vector.new({0.05, 0, 0}),
  ['x'] = vector.new({-.05, 0, 0}),
  ['a'] = vector.new({0, 0.05, 0}),
  ['d'] = vector.new({0, -.05, 0}),
  ['q'] = vector.new({0, 0, 15*math.pi/180}),
  ['e'] = vector.new({0,0, -15*math.pi/180}),
}


local char_to_state = {
  ['='] = 1,
  ['-'] = -1,  
}

--[[

local char_to_lfinger = {
  ['t'] = vector.new({-5,-5}),
  ['g'] = vector.new({0,0}),
  ['b'] = vector.new({5,5}),
}
--]]


local char_to_rfinger = {
--  ['t'] = vector.new({-10,-10,10}),--close
  ['t'] = vector.new({-10,-10,10}),--close
--  ['r'] = vector.new({-10,-50,10}),--close
  ['r'] = vector.new({-10,-100,10}),--close
  ['g'] = vector.new({0,0,0}),
  ['b'] = vector.new({10,10,-10}), --open
}

local function print_override()
  print( util.color('Override:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      override_target[1],
      override_target[2],
      override_target[3],
      override_target[4]*180/math.pi,
      override_target[5]*180/math.pi))

end



local function update(key_code)
  if type(key_code)~='number' or key_code==0 then return end
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)


  if key_char_lower==("1") then      body_ch:send'init'
	elseif key_char_lower==("0") then      body_ch:send'uninit'


  elseif key_char_lower==("5") then
    print( util.color("Fine control mode",'green')) 
    multiplier = 0.25
  elseif key_char_lower==("6") then
    print( util.color("Coarse control mode",'red'))     
    multiplier = 1

	elseif key_char_lower==("8") then  
		if mcm.get_walk_ismoving()>0 then 
			print("requesting stop")
			mcm.set_walk_stoprequest(1) 
		end
  elseif key_char_lower==("2") then  arm_ch:send'teleop'  
  elseif key_char_lower==("k") then  override_target=vector.new({0,0,0,  0,0,0,0})
  elseif key_char_lower==(" ") then
    hcm.set_move_target(movement_target)
		body_ch:send'approach' --todo
    movement_target = vector.zeros(3)

--    override_target = vector.zeros(6)
--    hcm.set_state_override(override_target)        
  end

  
  --notify target transform change
  local trmod = char_to_override[key_char_lower]
  if trmod and hcm.get_state_proceed()==0 then
    local trRArmTarget = vector.new(hcm.get_hands_right_tr_target() )
    hcm.set_hands_right_tr_target_old(trRArmTarget)
    trRArmTarget = trRArmTarget+trmod*multiplier
    print( util.color('Hand target: ','blue'), 
    	util.print_transform(trRArmTarget) )
    hcm.set_hands_right_tr_target(trRArmTarget)
    hcm.set_state_proceed(2)
    return
  end

--[[
  local lf = char_to_lfinger[key_char_lower]
  if lf then
    Body.move_lgrip1(lf[1])
    Body.move_lgrip2(lf[2])
    return
  end
--]]

  local rf = char_to_rfinger[key_char_lower]
  if rf then
    Body.set_rgrip_command_torque(rf)
    return
  end



  local state_adj = char_to_state[key_char_lower]
  if state_adj then
    print( util.color('State advance','yellow'), state_adj )
    hcm.set_state_proceed(state_adj)
    return
  end

 	local movetarget_adj = char_to_movetarget[key_char_lower]
 	if movetarget_adj then
 		movement_target=movement_target+movetarget_adj
 		print( util.color('Move target: ','yellow'), 
 			string.format("%.2f %.2f %d",movement_target[1],movement_target[2],movement_target[3]*180/math.pi )
 			)
 		return
 	end
end



if ... and type(...)=='string' then
  WAS_REQUIRED = true
  return {entry=nil, update=update, exit=nil}
end

local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.block()
  update(key_code)
end
