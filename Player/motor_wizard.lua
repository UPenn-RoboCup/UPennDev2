-----------------------------------------------------------------
-- Dynamixel Motor Communication
-- Performs callbacks on read/writes to the chain
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi

-- Libraries
local unix = require'unix'
local signal = require'signal'
local libDynamixel = require'libDynamixel'
local Body = require'Body'
local joint_to_motor = Body.servo.joint_to_motor
local motor_to_joint = Body.servo.motor_to_joint

-- Body entry
Body.entry()

-- Setup the dynamixels array
local dynamixels = {}

-- Initialize the dynamixels
--local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5A')
local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5B')
--local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5C')
--local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5D')
--local spine_dynamixel = libDynamixel.new_bus()

-- Spine dynamixel
if spine_dynamixel then
  spine_dynamixel.name = 'Spine'
  spine_dynamixel.data = {}
  table.insert(dynamixels,spine_dynamixel)
  local last_spine = 0
  local spine_cnt = 0
  -- Set up the callback when joints were read
  spine_dynamixel.callback = function(data)
    
    os.execute('clear')
    -- Update the shared memory
    for k,v in pairs(data) do
      -- k is the motor id
      -- v is the step value
      local idx = motor_to_joint[k]
      local rad = Body.make_joint_radian( idx, v )
      local deg = rad*RAD_TO_DEG
      print( string.format('Joint %d @ %.2f, step: %d',k,deg,v) )
      Body.set_sensor_position( rad, idx )
    end
    
    -- If finished a read, then stop the reading process
    spine_dynamixel.perform_read = false
    
    -- Show debugging output
    local t_diff = spine_dynamixel.t_last_read - last_spine
    spine_cnt = spine_cnt+1
    if t_diff>1 then
      local spine_fps = spine_cnt / t_diff
      local spine_debug = string.format('%s chain reading at %.2f Hz',
      spine_dynamixel.name,spine_fps)
      print()
      print(spine_debug)
      spine_cnt = 0
      last_spine = spine_dynamixel.t_last_read
    end
    end-- callback function
end -- if spine chain

-- Begin to service
assert(#dynamixels>0,"No dynamixels detected!")
io.write('Servicing ',#dynamixels,' Dynamixel chains.\n\n')
io.flush()

local main = function()
  local main_cnt = 0
  local t0 = unix.time()
  
  -- Entry should torque on the motors...
  Body.set_aux_torque_enable(0)
  local sync_torque = Body.set_aux_torque_enable_packet()
  --print(sync_torque:byte(1,#sync_torque))
  table.insert( spine_dynamixel.instructions, sync_torque )  
  
  -- Enter the coroutine
  while true do
    
    --------------------
    -- Set commands for next sync
    if #spine_dynamixel.instructions==0 then
      local sync_aux = Body.set_aux_command_packet()
      --print(sync_aux:byte(1,#sync_aux))
      --table.insert( spine_dynamixel.instructions, sync_aux )
    end
    
    -- Update the read every so often
    if unix.time()-spine_dynamixel.t_last_read > 1 then
      spine_dynamixel.perform_read = true
    end
    
    --------------------
    -- Show debugging information
    main_cnt = main_cnt + 1
    local t_now = unix.time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      for i,d in ipairs(dynamixels) do
        local last_seen = t_now-math.max(d.t_last_read,d.t_last_write)
        debug_str = debug_str..string.format(
        '\n\tDynamixel %s chain was seen %5.3f seconds ago',
        d.name,last_seen)
      end
      --[[
      os.execute('clear')
      print(debug_str)
      --]]
      t0 = t_now
      main_cnt = 0
    end
    
    --------------------
    -- Do not yield anything for now
    coroutine.yield()
    
  end
end

--------------------
-- Print the motor state upon shutdown
function shutdown()
  print'Shutting down the Dynamixel chains...'
  
  for i,d in ipairs(dynamixels) do
    -- Turn off motors
    -- TODO: Store the motor type for each on the chain
    -- NOTE: This could be used as a check on the body configuration
    libDynamixel.set_mx_torque_enable( d.ids_on_bus, 0, d )
    Body.set_aux_torque_enable(0)
    libDynamixel.set_mx_led( d.ids_on_bus, 0, d )
    -- Print out the chain
    io.write('\n',d.name,' chain\n')
    for _,m in ipairs(d.ids_on_bus) do
      local j = motor_to_joint[m]
      local rad = Body.get_joint_position(j)
      local deg = rad * RAD_TO_DEG
      local msg = string.format('%s Joint %d (Motor %d): %.3f rad %.1f deg',
      Body.inv_parts[j], j,m,rad,deg)
      print(msg)
    end
    d:close()
  end
  error('Finished!')
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

libDynamixel.service( dynamixels, main )