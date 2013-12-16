dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'Body',
  'unix',
  'util',
  'vector',
  'torch',
  'getch',
  'msgpack'
}

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end

-- FSM communicationg
local listing = unix.readdir(HOME..'/Player')
-- Add all FSM directories that are in Player
local simple_ipc = require'simple_ipc'
local fsm_ch_vars = {}
for _,sm in ipairs(listing) do
  local found = sm:find'FSM'
  if found then
    -- make GameFSM to game_ch
    local name = sm:sub(1,found-1):lower()..'_ch'
    table.insert(fsm_ch_vars,name)
    -- Put into the global space
    _G[name] = simple_ipc.new_publisher(sm,true)
  end
end

-- Shared memory
local listing = unix.readdir(HOME..'/Memory')
local shm_vars = {}
for _,mem in ipairs(listing) do
  local found, found_end = mem:find'cm'
  if found then
    local name = mem:sub(1,found_end)
    table.insert(shm_vars,name)
    require(name)
  end
end

-- RPC engine
rpc_ch = simple_ipc.new_requester(Config.net.reliable_rpc)

-- Mesh requester
mesh_req_ch = simple_ipc.new_requester(Config.net.reliable_mesh)

-- Useful constants
DEG_TO_RAD = Body.DEG_TO_RAD
RAD_TO_DEG = Body.RAD_TO_DEG

print( util.color('FSM Channel','yellow'), table.concat(fsm_ch_vars,' ') )
print( util.color('SHM access','blue'), table.concat(shm_vars,' ') )


local channels = {
  ['motion_ch'] = motion_ch,
  ['body_ch'] = body_ch,
  ['arm_ch'] = arm_ch,
}


-- Events for the FSMs
local char_to_event = {
  ['1'] = {'body_ch','init'},

  ['2'] = {'arm_ch','toolgrab'},
  ['3'] = {'arm_ch','doorgrab'},
  ['4'] = {'arm_ch','pushdoorgrab'},
  ['5'] = {'arm_ch','loaddoorgrab'},


  ['6'] = {'arm_ch','smallvalvegrab'},
  ['7'] = {'arm_ch','barvalvegrab'},

  ['8'] = {'arm_ch','smallvalveleftgrab'},


  ['9'] = {'arm_ch','hosegrab'},
  ['0'] = {'arm_ch','debrisgrab'},

  ['r'] = {'arm_ch','rocky'},
--  ['t'] = {'arm_ch','teleop'},
--  ['y'] = {'arm_ch','test'},

 
}

local char_to_override = {
  ['i'] = vector.new({0.01, 0, 0,   0,0,0,0}),
  [','] = vector.new({-.01, 0, 0,   0,0,0,0}),
  ['j'] = vector.new({0, 0.01, 0,   0,0,0,0}),
  ['l'] = vector.new({0, -.01, 0,   0,0,0,0}),
  ['u'] = vector.new({0, 0, 0.01,  0,0,0,0}),
  ['m'] = vector.new({0,0, -.01,   0,0,0,0}),
  
  --Yaw
  ['h'] = vector.new({0,0,0,     0,1,0,0}),
  [';'] = vector.new({0,0,0,    0,-1,0,0}),

  --Pitch
  ['y'] = vector.new({0,0,0,     -1,0,0,0}),
  ['n'] = vector.new({0,0,0,     1,0,0,0}),

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



local function send_command_to_ch(channel, cmd_string)
  -- Default case is to send the command and receive a reply
--  local ret   = channel:send(msgpack.pack(cmd))
--  local reply = channel:receive()
--  return msgpack.unpack(reply)
print(cmd_string)
  local ret   = channel:send(cmd_string)
  return
end

local function process_character(key_code,key_char,key_char_lower)
  local cmd

  -- Send motion fsm events
  local event = char_to_event[key_char_lower]
  if event then
    print( event[1], util.color(event[2],'yellow') )    
    return send_command_to_ch(channels[event[1]],event[2])
  end
  
  --notify target transform change
  local trmod = char_to_override[key_char_lower]
  if trmod then
    local override_old = hcm.get_state_override()
    local tr = vector.new(trmod) + vector.new(override_old)
    print( util.color('Override:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      tr[1],
      tr[2],
      tr[3],
      tr[4]*180/math.pi,
      tr[5]*180/math.pi))
    hcm.set_state_override_target(tr)    
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




------------
-- Start processing
--os.execute("clear")
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
