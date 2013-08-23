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
local vector  = require'vector'
local Body  = require'Body'
-- Keypresses for walking
local simple_ipc = require'simple_ipc'
local motion_events = simple_ipc.new_publisher('fsm_motion',true)
require'mcm'

local char_to_event = {
  ['7'] = 'sit',
  ['8'] = 'stand',
  ['9'] = 'walk',
}

local char_to_vel = {
  ['i'] = vector.new({0.1, 0, 0}),
  ['k'] = vector.new({-.1, 0, 0}),
  ['j'] = vector.new({0, 0.1, 0}),
  ['l'] = vector.new({0, -.1, 0}),
  ['u'] = vector.new({0, 0, 5})*math.pi/180,
  ['o'] = vector.new({0, 0, 5})*math.pi/180,
  ['0'] = function() mcm.set_walk_vel({0,0,0}) end
}

local function process_character(key_code,key_char,key_char_lower)
  local evt_str = char_to_event[key_char_lower]
  if evt_str then
    print( util.color(evt_str,'yellow') )
    motion_events:send(evt_str)
    return
  end

  local vel_adjustment = char_to_vel[key_char_lower]
  if type(vel_adjustment)=='table' then
    mcm.set_walk_vel(vel_adjustment+mcm.get_walk_vel())
    print( util.color('Inc vel by','yellow'), vel_adjustment, 'to', mcm.get_walk_vel() )
    return
  elseif type(vel_adjustment)=='function' then
    local cur = mcm.get_walk_vel()
    vel_adjustment()
    print( util.color('Vel from','yellow'), cur, 'to', mcm.get_walk_vel() )
    return
  end
  print( util.color('Bad event','red') )
end

------------
-- Start processing
os.execute("clear")
io.flush()
local t0 = unix.time()
while true do
  
  -- Grab the keyboard character
  local key_code = getch.block()
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)
  
  -- Process the character
  local msg = process_character(key_code,key_char,key_char_lower)
  
  -- Measure the timing
  local t = unix.time()
  local t_diff = t-t0
  t0 = t
  local fps = 1/t_diff
  
  -- Print is_debugging message
  if is_debug then
    print( string.format('\nKeyboard | Code: %d, Char: %s, Lower: %s',
    key_code,key_char,key_char_lower) )
    print('Response time:',t_diff)
  end
    
end