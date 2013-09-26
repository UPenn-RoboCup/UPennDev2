-----------------------------------------------------------------
-- Keyboard Wizard
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'
--local is_debug = true

-- Libraries
local Config  = require'Config'
local unix  = require'unix'
local getch = require'getch'
local mp    = require'msgpack'
local util  = require'util'
local vector  = require'vector'
local Body  = require'Body'
-- Keypresses for walking
local simple_ipc = require'simple_ipc'
--local motion_events = simple_ipc.new_publisher('fsm_motion',true)
local rpc_ch = simple_ipc.new_requester(Config.net.reliable_rpc)

-- Events for the FSMs
local char_to_event = {
  ['7'] = {'MotionFSM','sit'},
  ['8'] = {'MotionFSM','stand'},
  ['9'] = {'MotionFSM','walk'},
  ['p'] = {'MotionFSM','step'},
  
  --
  ['1'] = {'ArmFSM','init'},  
  ['2'] = {'ArmFSM','ready'}  ,
  ['3'] = {'ArmFSM','teleop'},
  ['4'] = {'ArmFSM','reset'},   
}

local current_arm = 'larm'
local current_grip = 'larm'


local delta_pos = .01 -- meters
local delta_ang = 2.5*Body.DEG_TO_RAD -- degrees
local char_to_ik = {
  -- Positions
  ['w'] = delta_pos*vector.new({1,0,0,  0,0,0}),
  ['x'] = delta_pos*vector.new({-1,0,0, 0,0,0}),
  ['a'] = delta_pos*vector.new({0,1,0,  0,0,0}),
  ['d'] = delta_pos*vector.new({0,-1,0, 0,0,0}),
  ['q'] = delta_pos*vector.new({0,0,1,  0,0,0}),
  ['z'] = delta_pos*vector.new({0,0,-1, 0,0,0}),
  -- Angles
  ['i'] = delta_ang*vector.new({0,0,0, 0,1,0 }),
  [','] = delta_ang*vector.new({0,0,0, 0,-1,0}),
  
  ['j'] = delta_ang*vector.new({0,0,0, 0,0,1 }),--Yaw
  ['l'] = delta_ang*vector.new({0,0,0, 0,0,-1}),
  
  ['u'] = delta_ang*vector.new({0,0,0, 1,0,0 }),
  ['m'] = delta_ang*vector.new({0,0,0, -1,0,0}),
}


local function send_command(cmd)
  -- Default case is to send the command and receive a reply
  local ret   = rpc_ch:send(mp.pack(cmd))
  local reply = rpc_ch:receive()
  return mp.unpack(reply)
end

local function process_character(key_code,key_char,key_char_lower)
  local cmd

  -- Send motion fsm events
  local event = char_to_event[key_char_lower]
  if event then
    print( event[1], util.color(event[2],'yellow') )
    cmd = {}
    cmd.fsm = event[1]
    cmd.evt = event[2]
    return send_command(cmd)
  end

  -- IK Changes with wasd/ijkl
  local ik_change = char_to_ik[key_char_lower]
  if ik_change then
    local cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'joints'
    cmd.key = 'p'..current_arm
    cmd.delta = ik_change
    send_command(cmd)
    print( util.color('Move '..current_arm,'yellow'), ik_change )
    return
  end
end

------------
-- Start processing
os.execute("clear")
io.flush()
local t0 = unix.time()

-------------------------------
-- Initialize HSM here
cmd = {}
cmd.shm = 'hcm'
cmd.segment = 'joints'
cmd.key = 'teleop'
cmd.val = 2 --IK MODE
send_command(cmd)

----------------------------------

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
