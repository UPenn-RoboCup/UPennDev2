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
  ['q'] = {'motion_ch','bias'},
  ['2'] = {'body_ch','stepplan'},
  ['3'] = {'body_ch','stepplan2'},

}

local servo_names={
  "hipyaw",
  "hiproll",
  "hippitch",
  "kneepitch",
  "anklepitch",
  "ankleroll"
}

local selected_servo = 1


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


  local bias_mag = 0.25*Body.DEG_TO_RAD
  local legBias = mcm.get_leg_bias()
  if key_char_lower=="[" then
    selected_servo = selected_servo-1
    if selected_servo<1 then selected_servo = 6 end    
  elseif key_char_lower=="]" then
    selected_servo = selected_servo+1
    if selected_servo>6 then selected_servo = 1 end
  
  elseif key_char_lower=="j" then
    legBias[selected_servo]=legBias[selected_servo]-bias_mag
  elseif key_char_lower=="l" then
    legBias[selected_servo]=legBias[selected_servo]+bias_mag
  elseif key_char_lower=="i" then    
    legBias[selected_servo+6]=legBias[selected_servo+6]-bias_mag
  elseif key_char_lower=="," then        
    legBias[selected_servo+6]=legBias[selected_servo+6]+bias_mag
  elseif key_char_lower=="0" then
    print(string.format("Current bias: \n%.2f %.2f %.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f %.2f %.2f ",
      unpack(vector.new(legBias)*Body.RAD_TO_DEG)))
  end
  mcm.set_leg_bias(legBias)
  print(servo_names[selected_servo]," : ",
    legBias[selected_servo]*Body.RAD_TO_DEG,
    legBias[selected_servo+6]*Body.RAD_TO_DEG
    )

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


