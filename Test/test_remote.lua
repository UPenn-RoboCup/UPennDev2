#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local WAS_REQUIRED = ... and type(...)=='string'
-- Look up tables
local code_lut, char_lut, lower_lut = {}, {}, {}
-- 
char_lut['1'] = function()
  motion_ch:send'stand'
end
lower_lut['r'] = function()
  motion_ch:send'remote'
end
lower_lut['q'] = function()
  motion_ch:send'quit'
end
--
local nleg = #Body.get_lleg_position()
local gain_leg = 0 -- left
code_lut[92] = function()
  -- Backslash
  gain_leg = 1 - gain_leg
end
local gain_joint = 1
char_lut[']'] = function()
  gain_joint = gain_joint + 1
  gain_joint = math.max(1, math.min(gain_joint, nleg))
end
char_lut['['] = function()
  gain_joint = gain_joint - 1
  gain_joint = math.max(1, math.min(gain_joint, nleg))
end
local pgLLeg = Body.get_lleg_position_p()
local pgRLeg = Body.get_rleg_position_p()
char_lut['='] = function()
  if gain_leg==0 then
    local gains = pgLLeg
    local pg0 = gains[gain_joint]
    pg0 = pg0 * 2
    gains[gain_joint] = math.max(1, math.min(pg0, 64))
  else
    local gains = pgRLeg
    local pg0 = gains[gain_joint]
    pg0 = pg0 * 2
    gains[gain_joint] = math.max(1, math.min(pg0, 64))
  end
end
char_lut['-'] = function()
  if gain_leg==0 then
    local gains = pgLLeg
    local pg0 = gains[gain_joint]
    pg0 = pg0 / 2
    gains[gain_joint] = math.max(1, math.min(pg0, 64))
  else
    local gains = pgRLeg
    local pg0 = gains[gain_joint]
    pg0 = pg0 / 2
    gains[gain_joint] = math.max(1, math.min(pg0, 64))
  end
end
-- Resend to the joints, in case of dropped packet
char_lut['0'] = function()
  Body.set_lleg_position_p(pgLLeg)
  Body.set_rleg_position_p(pgRLeg)
  -- Gains
  --pgLLeg = Body.get_lleg_position_p()
  --pgRLeg = Body.get_rleg_position_p()
end

local function show_status()
  if not IS_WEBOTS then os.execute('clear') end
  print(util.color('Remote Control', 'magenta'))
  print('Motion:', util.color(gcm.get_fsm_Motion(), 'green'))
  
	print('0: Write P Gains')
  local l_gain_indicator = vector.zeros(#pgLLeg)
  l_gain_indicator[gain_joint] = gain_leg==0 and 1 or 0
  local r_gain_indicator = vector.zeros(#pgLLeg)
  r_gain_indicator[gain_joint] = gain_leg==1 and 1 or 0
  
  print(string.format('%s %s\n%s\n%s',
    util.color('Left Leg P Gains', 'yellow'),
    gain_leg==0 and '*' or '',
    tostring(pgLLeg),
    l_gain_indicator
    )
  )
  print(string.format('%s %s\n%s\n%s',
    util.color('Right Leg P Gains', 'yellow'),
    gain_leg==1 and '*' or '',
    tostring(pgRLeg),
    r_gain_indicator
    )
  )
  
end

local function update(key_code)
  -- Assure that we got a character
  if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
  
  local code_f, char_f, lower_f = code_lut[key_code], char_lut[key_char], lower_lut[key_char_lower]
  --print('Codes', key_code, key_char, key_char_lower)
  -- Precedence
  if type(code_f)=='function' then
    code_f()
  elseif type(char_f)=='function' then
    char_f()
  elseif type(lower_f)=='function' then
    lower_f()
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
