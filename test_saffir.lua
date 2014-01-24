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
  'msgpack',
  'udp'
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

-- UDP communicating
local udp_sender = udp.new_sender('127.0.0.1', Config.net.saffir )
local udp_receiver = udp.new_receiver( Config.net.saffir )
print('UDP | Receiving on', udp_receiver)
local udp_poll = {}
udp_poll.socket_handle = udp_receiver:descriptor()
udp_poll.callback = process_udp
local channel_poll = simple_ipc.wait_on_channels( {udp_poll} )
local poll_timeout = 500   --2Hz


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

  ['2'] = {'arm_ch','firesuppress'},
  ['3'] = {'arm_ch','smallvalvegrab'},
  ['4'] = {'arm_ch','barvalvegrab'},
  ['5'] = {'arm_ch','hosegrab'},

  ['6'] = {'body_ch', 'stepover'},
}

local char_to_override = {
  ['i'] = vector.new({0.01, 0, 0,   0,0,0,0}),
  [','] = vector.new({-.01, 0, 0,   0,0,0,0}),
  ['j'] = vector.new({0, 0.01, 0,   0,0,0,0}),
  ['l'] = vector.new({0, -.01, 0,   0,0,0,0}),
  ['u'] = vector.new({0, 0, 0.01,  0,0,0,0}),
  ['m'] = vector.new({0,0, -.01,   0,0,0,0}),
  
  --Yaw
  ['h'] = vector.new({0,0,0,     0,0,5,0}),
  [';'] = vector.new({0,0,0,    0,0,-5,0}),

  --Pitch
  ['y'] = vector.new({0,0,0,     0,5,0, 0}),
  ['n'] = vector.new({0,0,0,     0,-5,0,  0}),

  --Task
  ['['] = vector.new({0,0,0,     -1,0,0,-1}),
  [']'] = vector.new({0,0,0,     1,0,0,1}),
}

local char_to_state = {
  ['='] = 1,
  ['-'] = -1,  
}

local char_to_rfinger = {}
local char_to_lfinger = {}

local char_to_tilt_pan = {
  --['s'] = '2,5',
  ['w'] = '0,0.1',  -- for HEAD: yaw, pitch
  ['x'] = '0,-0.1',
  ['a'] = '0.1,0',
  ['d'] = '-0.1,0',
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

  -- Send message over UDP
  local event = char_to_tilt_pan[key_char_lower]
  if event then
    print( util.color(event,'yellow') )    
    local ret = udp_sender:send(event)
    if ret==#event then
    	print('Sent '..ret..' bytes out of '..#event)
    end
    return ret
  end

  -- Send motion fsm events
  local event = char_to_event[key_char_lower]
  if event then
    print( event[1], util.color(event[2],'yellow') )    
    return send_command_to_ch(channels[event[1]],event[2])
  end
  
  --notify target transform change
  local trmod = char_to_override[key_char_lower]
  if trmod then
    --[[
    local override_old = hcm.get_state_override()
    local tr = vector.new(trmod) + vector.new(override_old)
    --]]
    local tr = trmod
    print( util.color('Override:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f %.1f",
      tr[1],
      tr[2],
      tr[3],
      tr[4],  -- The turnUnit is task-specific
      tr[5],
      tr[6]))
    --hcm.set_state_override_target(tr)    
    hcm.set_state_override(tr)    
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


local function process_udp()
	while udp_receiver:size()>0 do
		local data = udp_receiver:receive()
		print( util.color('Move hand to:','green'), data )
	end
end

------------
-- Start processing
--os.execute("clear")
io.flush()
local t0 = unix.time()
--local is_debug = true
while true do
	-- UDP message passing
	--local npoll = channel_poll:poll( poll_timeout )
	while udp_receiver:size()>0 do
		local data = udp_receiver:receive()
		local comma = data:find(',')
    local pitch = tonumber( data:sub(1,comma-1) )
    local yaw = tonumber( data:sub(comma+1,#data) )
    -- If in RADIAN
    --local target_pose = vector.new({pitch, yaw})  -- yaw, pitch for HEAD
    -- If in DEGREE
    local target_pose = vector.new({pitch*DEG_TO_RAD, yaw*DEG_TO_RAD})

    -- For moving the head
    --[[ If sending incremental amount
    local head_angle = Body.get_head_command_position()
    target_pose[1] = target_pose[1] + head_angle[1]
    target_pose[2] = target_pose[2] + head_angle[2]
    --]]
    hcm.set_motion_headangle(target_pose)
		print( util.color('Move head to:','green'), target_pose[1]*RAD_TO_DEG, target_pose[2]*RAD_TO_DEG )
    
    --[[ For moving the right arm
    local override = {0,0,0, 0, target_pose[1], target_pose[2], 0}
    hcm.set_state_override(override)
		print( util.color('Move hand to:','green'), unpack(target_pose) )
		--]]
	end

  --[[ Grab the keyboard character
  local key_code = getch.block()
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)
  
  -- Process the character
  local msg = process_character(key_code,key_char,key_char_lower)
  --]]

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

udp_receiver:close()