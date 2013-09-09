-----------------------------------
-- Dynamixel Motor Communication --
-- Performs callbacks for        --
-- libDynamixel Servicing        --
-- (c) Stephen McGill, 2013      --
-----------------------------------
dofile'include.lua'

-- Libraries
local Body   = require'Body'
local signal = require'signal'
local vector = require'vector'
local util   = require'util'
local libDynamixel = require'libDynamixel'
-- Easy access to useful elements
local joint_to_motor = Body.servo.joint_to_motor
local motor_to_joint = Body.servo.motor_to_joint
local jointNames = Body.jointNames
local DEG_TO_RAD = Body.DEG_TO_RAD
local RAD_TO_DEG = Body.RAD_TO_DEG
require'jcm'

--------------------
-- Setup helpful variables
local dynamixels = {}
local nMotors = 0
local status_color = {
  ['suspended'] = 'green',
  ['dead'] = 'red',
}
-- Lookup table for id to chain
local idx_to_dynamixel = {}
local idx_to_didx = {}
local idx_to_ids = {}
local idx_to_vals = {}

--[[
-- TODO: Battery (every second, update? via main routine...)
local function update_battery()
  nx_vol_now = Dynamixel:get_nx_voltage( nx_ids )
  --Average voltage reading from servos
  sum = 0;
  if nx_vol_now then
    for i=1,#nx_vol_now do sum=sum+nx_vol_now[i]; end
    return sum / #nx_vol_now / 10;
  end
  return 0;
end
--]]

--------------------
-- Callback processing on data from a named register
-- TODO: eliminate jcm, and use Body calls
-- TODO: that might be bad for performance, though
local update_read = function(self,data,register)
  local t = Body.get_time()
  -- Update the shared memory
  for k,v in pairs(data) do
    -- k is the motor id
    -- v is the register value
    -- Find the humanoid joint
    local idx = motor_to_joint[k]

    -- Debug
    print(string.format('%s: %s %s is %g',self.name,jointNames[idx],register,v))
    
    -- Specific handling of register types
    if register=='position' then
      local rad = Body.make_joint_radian( idx, v )
      jcm.sensorPtr.position[idx] = rad
      -- Kill the read once flag
      if jcm.readPtr.position[idx]==1 then jcm.readPtr.position[idx]=0 end
      -- Update the timestamp
      jcm.treadPtr.position[idx] = t
    elseif register=='load' then
      if v>=1024 then v = v - 1024 end
      local load_ratio = v/10.24
      jcm.sensorPtr.load[idx] = load_ratio
      -- Kill the read once flag
      if jcm.readPtr.load[idx]==1 then jcm.readPtr.load[idx]=0 end
      -- Update the timestamp
      jcm.treadPtr.load[idx] = t
    elseif register=='battery' then
      -- Somehow make an estimate
    end

    -- Update the read timestamps in shared memory
    -- TODO: eliminate jcm, and use a Body call
    local ptr = jcm.sensorPtr[register]
    if not ptr then
      print('Register is not in jcm',register)
      return
    end
    ptr[idx] = v
    ptr = jcm.readPtr[register]
    -- Kill the read once
    if ptr[idx]==1 then ptr[idx]=0 end
    -- Update the timestamp
    ptr = jcm.treadPtr[register]
    ptr[idx] = t

  end -- loop through data

end -- read callback function

--------------------
-- Initialize the dynamixels
local right_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5A')
local left_dynamixel  = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5B')
local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5C')

--------------------
-- Check the dynamixels
-- Left dynamixel
if left_dynamixel then
  left_dynamixel.name = 'LArm'
  table.insert(dynamixels,left_dynamixel)
  -- Set up the callback when joints were read
  left_dynamixel.callback = update_read
  left_dynamixel.mx_on_bus = {14,16,18}
  left_dynamixel.nx_on_bus = {1, 3, 5, 7, 9, 11}
  left_dynamixel.ids_on_bus = {1, 3, 5, 7, 9, 11,14,16,18}
end -- if left chain

-- Right dynamixel
if right_dynamixel then
  right_dynamixel.name = 'RArm'
  table.insert(dynamixels,right_dynamixel)
  -- Set up the callback when joints were read
  right_dynamixel.callback = update_read
  right_dynamixel.mx_on_bus = {15,17} --{13,15,17}
  right_dynamixel.nx_on_bus = {2, 4, 6, 8, 10, 12}
  right_dynamixel.ids_on_bus = {2, 4, 6, 8, 10, 12,13,15,17}
end -- if left chain

-- Spine dynamixel
if spine_dynamixel then
  spine_dynamixel.name = 'Spine'
  table.insert(dynamixels,spine_dynamixel)
  -- Set up the callback when joints were read
  spine_dynamixel.callback = update_read
  spine_dynamixel.mx_on_bus = {36}
  spine_dynamixel.nx_on_bus = {19,20,25,26,27,28}
  spine_dynamixel.ids_on_bus = {19,20,25,26,27,28,36}
end -- if spine chain

--------------------
-- Clean Shutdown function
function shutdown()
  print'Shutting down the Dynamixel chains...'
  
  for d_id,d in ipairs(dynamixels) do
    -- Torque off motors
    libDynamixel.set_mx_torque_enable( d.mx_on_bus, 0, d )
    libDynamixel.set_nx_torque_enable( d.nx_on_bus, 0, d )
    -- TODO: Save the torque enable states to SHM
    -- Close the fd
    d:close()
    -- Print helpful message
    print('Closed',d.name)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

--------------------
-- Dynamixel chain setup
-- Identify the MX/NX motor types on the chain
-- TODO: Verify with Body that the ids have the right motor type
local function entry()

  Body.entry()

  for didx,dynamixel in ipairs(dynamixels) do
    -- Some temporay variables
    dynamixel.packet_ids  = {}
    dynamixel.packet_vals = {}
    dynamixel.mx_packet_ids  = {}
    dynamixel.mx_packet_vals = {}


    if not dynamixel.ids_on_bus then
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
    end
    nMotors = nMotors + #dynamixel.ids_on_bus
    
    -- Read each motor id to classify each as NX or MX motors
    -- NOTE: NX and MX have same model register structure,
    -- so it is OK to use get_nx_ for both MX and NX
    -- TODO: There is a model difference on the right shoulder!
    if not dynamixel.mx_on_bus or not dynamixel.nx_on_bus then
      io.write('Checking motor model numbers...')
      io.flush()
      for _,id in ipairs(dynamixel.ids_on_bus) do
        local model_reg = libDynamixel.get_nx_model_num(id,dynamixel)
        assert(model_reg,string.format('Bad model! %d',id) )
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
    end
    print(#dynamixel.mx_on_bus..' MX:',unpack(dynamixel.mx_on_bus))
    print(#dynamixel.nx_on_bus..' NX:',unpack(dynamixel.nx_on_bus))
    
    -- TODO: Set the status return level on every startup?
    --local status = libDynamixel.set_nx_status_return_level(m,1,test_dynamixel)
    
    -- Perform a read to instantiate the motor commands and sensor positions
    for d_id,id in ipairs(dynamixel.nx_on_bus) do

      -- Read the NX motor positions
      local idx = motor_to_joint[id]
      local pos_status = libDynamixel.get_nx_position(id,dynamixel)
      assert(pos_status, string.format('%s: Did not find ID %s: %d',dynamixel.name,Body.jointNames[idx],id) )
      dynamixel.t_last_read = Body.get_time()

      -- Parse the value
      local pos_parser = libDynamixel.byte_to_number[ #pos_status.parameter ]
      local pos_val = pos_parser(unpack(pos_status.parameter))
      local rad = Body.make_joint_radian( idx, pos_val )

      -- Sync shared memory
      Body.set_sensor_position( rad, idx )
      Body.set_actuator_command_position( rad, idx )

      -- Populate the lookup table from id to chain
      idx_to_dynamixel[idx] = dynamixel
      idx_to_didx[idx] = didx
      idx_to_ids[idx]  = packet_ids
      idx_to_vals[idx] = packet_vals
      
      -- Debug output
      print( util.color(Body.jointNames[idx],'yellow'), '\n',
        string.format('\t%d (%d) @ %.2f, step: %d',
        idx,id,rad*Body.RAD_TO_DEG,pos_val) )

    end -- NX position sync
    
    -- Perform a read to instantiate the motor commands and sensor positions
    for _,id in ipairs(dynamixel.mx_on_bus) do
      local idx = motor_to_joint[id]
      local pos_status = libDynamixel.get_mx_position(id,dynamixel)
      assert(pos_status, string.format('%s: Did not find ID %s: %d',dynamixel.name,Body.jointNames[idx],id) )
      dynamixel.t_last_read = Body.get_time()

      -- Parse the value
      local pos_parser = libDynamixel.byte_to_number[ #pos_status.parameter ]
      local pos_val = pos_parser(unpack(pos_status.parameter))
      local rad = Body.make_joint_radian( idx, pos_val )

      -- Sync shared memory
      Body.set_sensor_position( rad, idx )
      Body.set_actuator_command_position( rad, idx )

      -- Populate the tables
      idx_to_dynamixel[idx] = dynamixel
      idx_to_didx[idx] = didx
      idx_to_ids[idx]  = mx_packet_ids
      idx_to_vals[idx] = mx_packet_vals
      
      -- Debug output
      print( util.color(Body.jointNames[idx],'yellow'), '\n',
        string.format('\t%d (%d) @ %.2f, step: %d',
        idx,id,rad*Body.RAD_TO_DEG,pos_val) )
    end -- MX position sync
  end -- for each dynamixel chain
end -- entry

--------------------
-- Update the commands to write to the robot
-- TODO: What if too many commands?
local update_commands = function()
  -- Loop through the registers
  for register,write_ptr in pairs(jcm.writePtr) do
    local val_ptr = jcm.actuatorPtr[register]
    -- Reset the tmp variable
    for _,d in ipairs(dynamixels) do
      d.packet_ids  = {}
      d.packet_vals = {}
      d.mx_packet_ids  = {}
      d.mx_packet_vals = {}
    end
    local set_func    = libDynamixel['set_nx_'..register]
    local mx_set_func = libDynamixel['set_mx_'..register]
    -- go through each value
    for idx,is_write in ipairs(write_ptr) do
      if is_write>0 then
        -- Add the write instruction to the chain
        local pkt_ids = idx_to_ids[idx]
        if pkt_ids then
          table.insert( pkt_ids, joint_to_motor[idx] )
          if register=='command_position' then
            -- Convert from radians to steps for set command_position
            table.insert( idx_to_vals[idx], make_joint_step(idx,val_ptr[idx]) )
          else
            table.insert( idx_to_vals[idx], val_ptr[idx] )
          end
        end
        -- Kill the write once
        if is_write==1 then write_ptr[idx]=0 end
      end
    end -- for each enable
    -- Make the request for this register on each chain
    for _,d in ipairs(dynamixels) do
      local nids = #d.packet_ids
      if #d.packet_ids>0 then
        -- Make the NX request
        table.insert( d.commands, 
          set_func(d.packet_ids,d.packet_vals) )
      end
      if #d.mx_packet_ids>0 then
        -- Make the MX request
        table.insert( d.commands, 
          mx_set_func(d.mx_packet_ids,d.mx_packet_vals) )
      end
    end -- for making commands for the chains
  end -- for each register
end

-- Set commands for next sync read    
-- Update the read every so often
local update_requests = function()
  -- Loop through the registers
  for register,read_ptr in pairs(jcm.readPtr) do
    -- Reset the tmp variable
    for _,d in ipairs(dynamixels) do
      d.packet_ids  = {}
      d.mx_packet_ids = {}
    end
    local get_func = libDynamixel['get_nx_'..register]
    local mx_get_func = libDynamixel['get_mx_'..register]
    for idx,is_read in ipairs(read_ptr) do
      -- Check if we are to read each of the values
      if is_read>0 then
        -- Add the read instruction to the chain
        local pkt_ids = idx_to_ids[idx]
        if pkt_ids then
          table.insert( pkt_ids, joint_to_motor[idx] )
        end
        -- Kill the read once
        if is_read==1 then read_ptr[idx]=0 end
      end
    end -- for each enable
    -- Make the request for this register on each chain
    for _,d in ipairs(dynamixels) do
      local nids    = #d.packet_ids
      local mx_nids = #d.mx_packet_ids
      if nids>0 then
        -- Make the request
        local req = {
          nids = nids,
          reg  = register,
          inst = get_func(d.packet_ids)
        }
        table.insert( d.requests, req )
      end
      if mx_nids>0 then
        -- Make the request
        local req = {
          nids = mx_nids,
          reg  = register,
          inst = mx_get_func(d.mx_packet_ids)
        }
        table.insert( d.requests, req )
      end
    end -- for making commands for the chains
  end -- for each register
end --function

--------------------
-- Begin the main routine
local led_state = 0
local main = function()
  local main_cnt = 0
  local t0 = Body.get_time()
  
  -- Enter the coroutine
  while true do
    local t = Body.get_time()
    
    -- Set commands for next sync write
    if #d.commands==0 then update_commands() end
    if #d.requests==0 then update_requests() end
    
    -- Show debugging information and blink the LED
    main_cnt = main_cnt + 1
    local t_diff = t - t0
    if t_diff>1 then
      local debug_tbl = {}
      table.insert(debug_tbl, string.format(
        '\nMain loop: %7.2f Hz (LED: %d)',
        main_cnt/t_diff,led_state))
      led_state = 1-led_state
      for _,d in ipairs(dynamixels) do
        -- Blink the led
        local sync_led_cmd = 
          libDynamixel.set_mx_led( d.mx_on_bus, led_state )
        local sync_led_red_cmd = 
          libDynamixel.set_nx_led_red( d.nx_on_bus, 255*led_state )
        table.insert( d.commands, sync_led_cmd )
        table.insert( d.commands, sync_led_red_cmd )

        -- Append debugging information
        local dstatus = coroutine.status(d.thread)
        table.insert(debug_tbl, util.color(
          string.format('%s chain %s',d.name, dstatus),
          status_color[dstatus]))
        table.insert(debug_tbl,string.format(
          '\n\tRead: %4.2f seconds ago\tWrite: %4.2f seconds ago',
          t-d.t_read,t-d.t_write))
        table.insert(debug_tbl,string.format(
          '\n\t%d requests in the pipeline',#d.requests))
        table.insert(debug_tbl,string.format(
          '\n\t%d commands in the pipeline',#d.commands))
      end
      os.execute('clear')
      print( table.concat(debug_str,'\n') )
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

os.exit()

print()
assert(#dynamixels>0,"No dynamixel buses!")
assert(nMotors>0,"No dynamixel motors found!")
print( string.format('Servicing %d dynamixel chains',#dynamixels) )
libDynamixel.service( dynamixels, main )