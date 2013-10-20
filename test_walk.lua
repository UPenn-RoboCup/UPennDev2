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


  ['7'] = {'motion_ch','sit'},
  ['8'] = {'motion_ch','stand'},
  ['9'] = {'motion_ch','walk'},

  ['t'] = {'body_ch','stepover'},
  ['y'] = {'body_ch','stepplan'},
  --
  ['a'] = {'arm_ch','init'},
  ['s'] = {'arm_ch','reset'},
  ['r'] = {'arm_ch','ready'},
  --
  ['x'] = {'arm_ch','teleop'},
  --
  ['w'] = {'arm_ch','wheelgrab'},
  --
  ['d'] = {'arm_ch','doorgrab'},
}

local char_to_vel = {
  ['i'] = vector.new({0.025, 0, 0}),
  [','] = vector.new({-.025, 0, 0}),
  ['h'] = vector.new({0, 0.025, 0}),
  [';'] = vector.new({0, -.025, 0}),
  ['j'] = vector.new({0, 0, 5})*math.pi/180,
  ['l'] = vector.new({0, 0, -5})*math.pi/180,
}

local char_to_wheel = {
  ['['] = -1*Body.DEG_TO_RAD,
  [']'] = 1*Body.DEG_TO_RAD,
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

  -- Adjust the velocity
  -- Only used in direct teleop mode
  local vel_adjustment = char_to_vel[key_char_lower]
  if vel_adjustment then
    print( util.color('Inc vel by','yellow'), vel_adjustment )
    local walk_vel_prev = mcm.get_walk_vel()
    local walk_vel_new = vector.new(walk_vel_prev)+vector.new(vel_adjustment)
    mcm.set_walk_vel(walk_vel_new)
    return
  elseif key_char_lower=='k' then
    print( util.color('Zero Velocity','yellow'))
    mcm.set_walk_vel({0,0,0})
    return
  end

  -- Adjust the wheel angle
  local wheel_adj = char_to_wheel[key_char_lower]
  if wheel_adj then
    print( util.color('Turn wheel','yellow'), wheel_adj )
    local turnAngle = hcm.get_wheel_turnangle() + wheel_adj
    hcm.set_wheel_turnangle(turnAngle)
    return
  elseif key_char_lower=='\\' then
    print( util.color('Center the wheel','yellow') )    
    hcm.set_wheel_turnangle(0)
    return
  end

--[[
  -- TODO: smarter range setting
  -- For now, care about things from 10cm to 1m in front
  local near, far = 0.10, 1
  if key_char_lower=='v' then
    print( util.color('Request Head Mesh','yellow') )
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'head_lidar'
    cmd.key = 'depths'
    cmd.val = {near,far}
    send_command(cmd)
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'head_lidar'
    cmd.key = 'net'
    --cmd.val = {1,1,0} --jpeg
    cmd.val = {1,2,0} --zlib
    return send_command(cmd)
  elseif key_char_lower=='c' then
    print( util.color('Request Chest Mesh','yellow') )
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'chest_lidar'
    cmd.key = 'depths'
    cmd.val = {near,far}
    send_command(cmd)

    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'chest_lidar'
    cmd.key = 'net'
    --cmd.val = {1,1,0} --jpeg
    cmd.val = {1,2,0} --zlib
    return send_command(cmd)
  end
  --]]
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
