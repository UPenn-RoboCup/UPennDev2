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

--------------------
-- Body entry
Body.entry()
-- Setup the dynamixels array
local dynamixels = {}
local nMotors = 0

--------------------
-- Clean Shutdown function
function shutdown()
  print'Shutting down the Dynamixel chains...'
  
  for i,d in ipairs(dynamixels) do
    -- Torque off motors
    libDynamixel.set_mx_torque_enable( d.mx_on_bus, 0, d )
    libDynamixel.set_nx_torque_enable( d.nx_on_bus, 0, d )
    -- Save the torque enable states to SHM
    Body.set_aux_torque_enable(0)
    -- Turn off the LEDs
    libDynamixel.set_mx_led( d.mx_on_bus, 0, d )
    libDynamixel.set_nx_led_red( d.nx_on_bus, 0, d )
    -- Close the fd
    d:close()
    -- Print helpful message
    print('Closed',d.name)
  end
  error('Finished shutdown procedure!')
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

--------------------
-- Dynamixel chain setup
-- Identify the MX/NX motor types on the chain
-- TODO: Verify with Body that the ids have the right motor type
local function entry()

  for _,dynamixel in ipairs(dynamixels) do
    -- Check which ids are on this chain
    io.write('\n')
    io.write('Checking ids on the ',dynamixel.name,' chain...')
    io.flush()
    dynamixel.ids_on_bus = dynamixel:ping_probe()
    -- TODO: For faster startup, cache the results in a Config/Body file
    -- dynamixel.ids_on_bus = {14,16,18}
    io.write'Done!\n'
    io.flush()
    if #dynamixel.ids_on_bus==0 then print('No motors on '..dynamixel.name) end
    nMotors = nMotors + #dynamixel.ids_on_bus
    
    -- Read each motor id to classify each as NX or MX motors
    -- NOTE: NX and MX have same model register structure,
    -- so it is OK to use get_nx_ for both MX and NX
    -- TODO: There is a model difference on the right shoulder!
    io.write('Checking motor model numbers...')
    io.flush()
    for _,id in ipairs(dynamixel.ids_on_bus) do
      local model_reg = libDynamixel.get_nx_model_num(id,dynamixel)
      local model_parser = libDynamixel.byte_to_number[ #model_reg.parameter ]
      local model_version = model_parser(unpack(model_reg.parameter))
      if model_version==29 then
        table.insert( dynamixel.mx_on_bus, model_reg.id )
      else
        table.insert( dynamixel.nx_on_bus, model_reg.id )
      end
    end
    io.write'Done!\n'
    io.flush()
    print(#dynamixel.mx_on_bus..' MX:',unpack(dynamixel.mx_on_bus))
    print(#dynamixel.nx_on_bus..' NX:',unpack(dynamixel.nx_on_bus))
    dynamixel.t_last_read = unix.time()
  end
end

--------------------
-- Callback processing on data from a named register
local update_read = function(self,data,name)
  --os.execute('clear')
  -- TODO: Utilize the name information, and not assume it is position data
  -- Update the shared memory
  for k,v in pairs(data) do
    -- k is the motor id
    -- v is the step value
    local idx = motor_to_joint[k]
    if k == 15 then print('finger',idx,k,v) end
    if name=='position' then
      local rad = Body.make_joint_radian( idx, v )
      --print( string.format('Joint %d @ %.2f, step: %d',k,rad,v) )
      Body.set_sensor_position( rad, idx )
    elseif name=='load' then
      print( string.format('Joint %d @ %d load',k,v) )
      --Body.set_sensor_position( rad, idx )
    else
      print('Could not place',name,'into shared memory')
    end
  end
    
  -- Show debugging output
  --[[
  local debug_msg = string.format(
  '%s chain reading %d packets %d ms %f', 
  self.name, #data, self.t_diff_read*1000, self.t_last_read)
  print()
  print(debug_msg)
  --]]
end -- callback function

--------------------
-- Update the commands to write to the robot
local update_instructions = function()
  -- Loop through the dynamixels and add instructions to send
  for _,d in ipairs(dynamixels) do
    -- Do not overpopulate (just yet)
    if #d.instructions==0 then
      -- Form the packets
      local sync_aux = Body.set_aux_command_packet()
      --print(sync_aux:byte(1,#sync_aux))
      table.insert( d.instructions, sync_aux )
    end
  end
end

-- Set commands for next sync read    
-- Update the read every so often
local update_requests = function()
  local t = unix.time()
  -- Loop through the dynamixels and add read requests
  for _,d in ipairs(dynamixels) do
    -- Do not overpopulate (just yet)
    if #d.requests==0 and t-d.t_last_read > 1 then
      local inst_nx = libDynamixel.get_nx_position( d.nx_on_bus )
      table.insert( d.requests, {inst_nx,'position'} )
      local inst_mx = libDynamixel.get_mx_position( d.mx_on_bus )
      table.insert( d.requests, {inst_mx,'position'} )
      --[[
      local inst_mx = libDynamixel.get_mx_load( d.mx_on_bus )
      table.insert( d.requests, {inst_mx,'load'} )
      --]]
    end
  end
end

--------------------
-- Initialize the dynamixels
local right_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5A')
local left_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5B')
local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5C')
--local none_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5D')
--local test_dynamixel = libDynamixel.new_bus()

--------------------
-- Left dynamixel
if left_dynamixel then
  left_dynamixel.name = 'LArm'
  table.insert(dynamixels,left_dynamixel)
  -- Set up the callback when joints were read
  left_dynamixel.callback = update_read
end -- if left chain

-- Right dynamixel
if right_dynamixel then
  right_dynamixel.name = 'RArm'
  table.insert(dynamixels,right_dynamixel)
  -- Set up the callback when joints were read
  right_dynamixel.callback = update_read
end -- if left chain

-- Spine dynamixel
if spine_dynamixel then
  spine_dynamixel.name = 'Spine'
  table.insert(dynamixels,spine_dynamixel)
  -- Set up the callback when joints were read
  spine_dynamixel.callback = update_read
end -- if spine chain

--------------------
-- Begin the main routine
local led_state = 0
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
    local t = unix.time()
    
    -- Set commands for next sync write
    update_instructions()
    update_requests()
    
    -- Show debugging information and blink the LED
    main_cnt = main_cnt + 1
    local t_diff = t - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      led_state = 1-led_state
      for _,d in ipairs(dynamixels) do
        --[[
        print()
        print(string.format(
        '%s chain |\tRead: %.2f Write: %.2f',
        d.name, t-d.t_last_read, t-d.t_last_write
        ))
        --]]
        -- Blink the led
        local sync_led_cmd = 
          libDynamixel.set_mx_led( d.mx_on_bus, led_state )
        local sync_led_red_cmd = 
          libDynamixel.set_nx_led_red( d.nx_on_bus, 255*led_state )
        table.insert( d.instructions, sync_led_cmd )
        table.insert( d.instructions, sync_led_red_cmd )
        
      end
      ----[[
      os.execute('clear')
      print(debug_str)
      --]]
      t0 = t
      main_cnt = 0
    end
    
    -- Do not yield anything for now
    coroutine.yield()
    
  end
end

--------------------
-- Start the service loop
entry()
print()
assert(#dynamixels>0,"No dynamixel buses!")
assert(nMotors>0,"No dynamixel motors found!")
print( string.format('Servicing %d dynamixel chains',#dynamixels) )
libDynamixel.service( dynamixels, main )