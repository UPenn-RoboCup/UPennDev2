-----------------------------------------------------------------
-- Dynamixel Motor Communication
-- Performs callbacks on read/writes to the chain
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi

-- Libraries
local Body   = require'Body'
local signal = require'signal'
local vector = require'vector'
local util   = require'util'
local libDynamixel = require'libDynamixel'
local joint_to_motor = Body.servo.joint_to_motor
local motor_to_joint = Body.servo.motor_to_joint

--------------------
-- Setup the dynamixels array
local dynamixels = {}
local nMotors = 0
local status_color = {
  ['suspended'] = 'green',
  ['dead'] = 'red',
}

--[[
-- TODO: Battery
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
local update_read = function(self,data,name)
  --os.execute('clear')
  -- TODO: Utilize the name information, and not assume it is position data
  -- Update the shared memory
  for k,v in pairs(data) do
    -- k is the motor id
    -- v is the step value
    local idx = motor_to_joint[k]
    -- Uncomment for specific bias tuning
    --if k == 15 then print('finger',idx,k,v) end
    if name=='position' then
      local rad = Body.make_joint_radian( idx, v )
      --print( string.format('Joint %d @ %.2f, step: %d',k,rad,v) )
      Body.set_sensor_position( rad, idx )
    elseif name=='load' then
      local load_ratio = v/10.24
      if v>=1024 then load_ratio = load_ratio - 100 end
      --print( string.format('Joint %d @ %d load (%.3f)',k,v,load_ratio) )
      Body.set_sensor_load( load_ratio, idx )
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
-- Initialize the dynamixels
local right_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5A')
local left_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5B')
local spine_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5C')
--local none_dynamixel = libDynamixel.new_bus('/dev/cu.usbserial-FTT3AAV5D')
--local test_dynamixel = libDynamixel.new_bus()

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
  
  for i,d in ipairs(dynamixels) do
    -- Torque off motors
    libDynamixel.set_mx_torque_enable( d.mx_on_bus, 0, d )
    libDynamixel.set_nx_torque_enable( d.nx_on_bus, 0, d )
    -- TODO: Save the torque enable states to SHM
    -- Turn off the LEDs
    libDynamixel.set_mx_led( d.mx_on_bus, 0, d )
    libDynamixel.set_nx_led_red( d.nx_on_bus, 0, d )
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

  for _,dynamixel in ipairs(dynamixels) do
    
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
    
    -- Set the status return level?
    --local status = libDynamixel.set_nx_status_return_level(m,1,test_dynamixel)
    
    -- Perform a read to instantiate the motor commands and sensor positions
    for _,id in ipairs(dynamixel.nx_on_bus) do
      local idx = motor_to_joint[id]
      local pos_status = libDynamixel.get_nx_position(id,dynamixel)
      assert(pos_status, string.format('%s: Did not find ID %s: %d',dynamixel.name,Body.jointNames[idx],id) )
      local pos_parser = libDynamixel.byte_to_number[ #pos_status.parameter ]
      local pos_val = pos_parser(unpack(pos_status.parameter))
      
      local rad = Body.make_joint_radian( idx, pos_val )
      print( util.color(Body.jointNames[idx],'yellow'), '\n',string.format('\t%d (%d) @ %.2f, step: %d',
        idx,id,rad*Body.RAD_TO_DEG,pos_val) )
      Body.set_sensor_position( rad, idx )
      Body.set_actuator_command_position( rad, idx )
      dynamixel.t_last_read = Body.get_time()
    end
    
    -- Perform a read to instantiate the motor commands and sensor positions
    for _,id in ipairs(dynamixel.mx_on_bus) do
      local idx = motor_to_joint[id]
      local pos_status = libDynamixel.get_mx_position(id,dynamixel)
      assert(pos_status, string.format('%s: Did not find ID %s: %d',dynamixel.name,Body.jointNames[idx],id) )
      local pos_parser = libDynamixel.byte_to_number[ #pos_status.parameter ]
      local pos_val = pos_parser(unpack(pos_status.parameter))
      
      local rad = Body.make_joint_radian( idx, pos_val )
      print( util.color(Body.jointNames[idx],'yellow'), '\n',string.format('\t%d (%d) @ %.2f, step: %d',
        idx,id,rad*Body.RAD_TO_DEG,pos_val) )
      Body.set_sensor_position( rad, idx )
      Body.set_actuator_command_position( rad, idx )
      dynamixel.t_last_read = Body.get_time()
    end
    
    -- Torque enable some motors
    -- TODO: this should be prettier...
    if dynamixel.name=='RArm' then
      local w_ids = vector.slice(joint_to_motor,Body.indexRArm,Body.indexRArm-1+Body.nJointRArm)
      print('Setting',w_ids,'on')
      local sync_en = libDynamixel.set_nx_torque_enable(w_ids,1)
      table.insert( dynamixel.instructions, sync_en )
    elseif dynamixel.name=='LArm' then
      local w_ids = vector.slice(joint_to_motor,Body.indexLArm,Body.indexLArm-1+Body.nJointLArm)
      print('Setting',w_ids,'on')
      local sync_en = libDynamixel.set_nx_torque_enable(w_ids,1)
      table.insert( dynamixel.instructions, sync_en )
    elseif dynamixel.name=='Spine' then
      local w_ids = vector.slice(joint_to_motor,Body.indexHead,Body.indexHead-1+Body.nJointHead)
      local sync_en = libDynamixel.set_nx_torque_enable(w_ids,1)
      print('Torque enable head',w_ids)
      table.insert( dynamixel.instructions, sync_en )
            
      local w_ids = vector.slice(joint_to_motor,Body.indexLidar,Body.indexLidar-1+Body.nJointLidar)
      local sync_en = libDynamixel.set_nx_torque_enable(w_ids,1)
      print('Torque enable lidar',w_ids)
      table.insert( dynamixel.instructions, sync_en )
    end
    dynamixel.t_last_write = Body.get_time()
  end
--  os.exit()
end

--------------------
-- Update the commands to write to the robot
local update_instructions = function()
  -- Loop through the dynamixels and add instructions to send
  for _,d in ipairs(dynamixels) do
    -- Do not overpopulate (just yet)
    if #d.instructions==0 then
      if d.name=='RArm' then
        local sync_rarm = Body.set_rarm_command_position_packet()
        table.insert( d.instructions, sync_rarm )
        -- Form the hand packets
        local sync_rgrip = Body.set_rgrip_command_position_packet()
        table.insert( d.instructions, sync_rgrip )
      elseif d.name=='LArm' then
        local sync_larm = Body.set_larm_command_position_packet()
        table.insert( d.instructions, sync_larm )
        local sync_lgrip = Body.set_lgrip_command_position_packet()
        table.insert( d.instructions, sync_lgrip )
      elseif d.name=='Spine' then
        local sync_lidar = Body.set_lidar_command_position_packet()
        table.insert( d.instructions, sync_lidar )
        local sync_head = Body.set_head_command_position_packet()
        table.insert( d.instructions, sync_head )
      end--d.name
    end--#instructions
  end
end

-- Set commands for next sync read    
-- Update the read every so often
local update_requests = function()
  local t = Body.get_time()
  -- Loop through the dynamixels and add read requests
  for _,d in ipairs(dynamixels) do
    -- Do not overpopulate (just yet)
    if #d.requests==0 and t-d.t_last_read>.5 then
      -- NX reading
      if #d.nx_on_bus>0 then
        local inst_nx = libDynamixel.get_nx_position( d.nx_on_bus )
        --table.insert( d.requests, {inst_nx,'position'} )
      end
      -- MX readings
      if #d.mx_on_bus>0 then
        for _,idx in ipairs(d.mx_on_bus) do
          local inst_mx = libDynamixel.get_mx_position( d.mx_on_bus )
          --table.insert( d.requests, {inst_mx,'position'} )
          --local inst_mx = libDynamixel.get_mx_load( idx )
          --table.insert( d.requests, {inst_mx,'load'} )
        end
      end -- if mx
    end --req==0
  end -- for d
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
    update_instructions()
    update_requests()
    
    -- Show debugging information and blink the LED
    main_cnt = main_cnt + 1
    local t_diff = t - t0
    if t_diff>1 then
      local debug_str = string.format(
        '\nMain loop: %7.2f Hz (LED: %d)',main_cnt/t_diff,led_state)
      led_state = 1-led_state
      for _,d in ipairs(dynamixels) do
        debug_str = debug_str..'\n\n'
        -- Blink the led
        local sync_led_cmd = 
          libDynamixel.set_mx_led( d.mx_on_bus, led_state )
        local sync_led_red_cmd = 
          libDynamixel.set_nx_led_red( d.nx_on_bus, 255*led_state )
        if #d.instructions<5 then
          table.insert( d.instructions, sync_led_cmd )
          table.insert( d.instructions, sync_led_red_cmd )
        end
        
        -- Append debugging information
        local t_read  = t-d.t_last_read
        local t_write = t-d.t_last_write
        local dstatus = coroutine.status(d.thread)
        debug_str = debug_str..util.color(string.format('%s chain %s',d.name, dstatus),status_color[dstatus])
        if d.message then debug_str = debug_str..': '..d.message end
        debug_str = debug_str..string.format(
          '\n\tRead: %4.2f seconds ago\tWrite: %4.2f seconds ago',t_read,t_write)
        debug_str = debug_str..string.format(
          '\n\t%d requests in the pipeline',#d.requests)
        debug_str = debug_str..string.format(
          '\n\t%d instructions in the pipeline',#d.instructions)
      end
      os.execute('clear')
      print(debug_str)
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