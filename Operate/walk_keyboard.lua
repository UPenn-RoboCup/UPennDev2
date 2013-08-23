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
-- Keypresses for walking
local simple_ipc = require'simple_ipc'
local motion_events = simple_ipc.new_publisher('fsm_motion',true)

local function process_character(key_code,key_char,key_char_lower)
  local evt_str
  if key_char_lower=='8' then
    evt_str = 'stand'
  end
  if evt_str then
    motion_events:send(evt_str)
    return evt_str
  end
  return'Bad event'
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
  
  -- http://lua-users.org/lists/lua-l/2009-12/msg00937.html
  -- Print result of the key press
  os.execute("clear")
  --print( state_msg() )
  --print()
  print(key_code..':', util.color(msg,'yellow'))
    
end