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


local char_to_override = {
--[[
  ['i'] = vector.new({0.01, 0, 0,   0,0,0,0}),
  [','] = vector.new({-.01, 0, 0,   0,0,0,0}),
  ['j'] = vector.new({0, 0.01, 0,   0,0,0,0}),
  ['l'] = vector.new({0, -.01, 0,   0,0,0,0}),
  ['u'] = vector.new({0, 0, 0.01,  0,0,0,0}),
  ['m'] = vector.new({0,0, -.01,   0,0,0,0}),
  --]]

  ['i'] = vector.new({0.04, 0, 0,   0,0,0,0}),
  [','] = vector.new({-.04, 0, 0,   0,0,0,0}),
  ['j'] = vector.new({0, 0.04, 0,   0,0,0,0}),
  ['l'] = vector.new({0, -.04, 0,   0,0,0,0}),
  ['u'] = vector.new({0, 0, 0.04,  0,0,0,0}),
  ['m'] = vector.new({0,0, -.04,   0,0,0,0}),
  



  --Yaw
  ['h'] = vector.new({0,0,0,     0,0,1,0}),
  [';'] = vector.new({0,0,0,    0,0,-1,0}),

  --Pitch
  ['y'] = vector.new({0,0,0,     0,-1,0, 0}),
  ['n'] = vector.new({0,0,0,     0,1,0,  0}),

  --Task
  ['['] = vector.new({0,0,0,     -1,0,0,-1}),
  [']'] = vector.new({0,0,0,     1,0,0,1}),
}

local char_to_state = {
  ['='] = 1,
  ['-'] = -1,  
}

local char_to_lfinger = {
  ['z'] = vector.new({-5,-5}),
  ['a'] = vector.new({0,0}),
  ['q'] = vector.new({5,5}),
}

local char_to_rfinger = {
  ['x'] = vector.new({-5,-5}),
  ['s'] = vector.new({0,0}),
  ['w'] = vector.new({5,5}),
}



local function update(key_code)
  if type(key_code)~='number' or key_code==0 then return end
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)


  if key_char_lower==("1") then      body_ch:send'init'
  elseif key_char_lower==("2") then  arm_ch:send'toolgrab'
  elseif key_char_lower==("3") then  arm_ch:send'pushdoorgrab'
  elseif key_char_lower==("4") then  arm_ch:send'teleop'
  elseif key_char_lower==("=") then      
    hcm.set_state_proceed(1)
  elseif key_char_lower==("-") then          
    hcm.set_state_proceed(-1)


  elseif key_char_lower==("k") then          
    override_target=vector.new({0,0,0,  0,0,0,0})
  elseif key_char_lower==(" ") then              

    hcm.set_state_override(override_target)    
  end


--[[
  ['2'] = {'arm_ch','toolgrab'},
  ['3'] = {'arm_ch','pushdoorgrab'},

  ['6'] = {'arm_ch','smallvalvegrab'},
  ['7'] = {'arm_ch','barvalvegrab'},

  ['9'] = {'arm_ch','hosegrab'},
  ['0'] = {'arm_ch','debrisgrab'},

  ['r'] = {'arm_ch','rocky'},
--]]

  
  --notify target transform change
  local trmod = char_to_override[key_char_lower]
  if trmod then
    --[[
    local override_old = hcm.get_state_override()
    local tr = vector.new(trmod) + vector.new(override_old)
    --]]
    override_target = override_target+trmod
    print( util.color('Override:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      override_target[1],
      override_target[2],
      override_target[3],
      override_target[4]*180/math.pi,
      override_target[5]*180/math.pi))
    --hcm.set_state_override_target(tr)    
    
    return
  end


  local lf = char_to_lfinger[key_char_lower]
  if lf then
    Body.move_lgrip1(lf[1])
    Body.move_lgrip2(lf[2])
    return
  end

  local rf = char_to_rfinger[key_char_lower]
  if rf then
    Body.move_rgrip1(rf[1])
    Body.move_rgrip2(rf[2])
    return
  end

 

  local state_adj = char_to_state[key_char_lower]
  if state_adj then
    print( util.color('State advance','yellow'), state_adj )
    hcm.set_state_proceed(state_adj)
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