-- Dynamixel Library
-- (c) 2013 Stephen McGill
-- (c) 2013 Daniel D. Lee
-- Support: http://support.robotis.com/en/product/dynamixel_pro/communication/instruction_status_packet.htm

local util = require'util'

local libDynamixel = {}
local DP1 = require'DynamixelPacket1' -- 1.0 protocol
local DP2 = require'DynamixelPacket2' -- 2.0 protocol
local unix = require'unix'
local stty = require'stty'
local using_status_return = true
-- 75ms default timeout
local READ_TIMEOUT = .0075

-- TODO: Make this a parameter to set externally
-- TODO: This should be tuned based on the byte size written?
--local WRITE_TIMEOUT = 1/250 -- 250Hz timeout
local WRITE_TIMEOUT = 1/125 -- 120Hz timeout
--local WRITE_TIMEOUT = 1/60 -- 60Hz timeout
--local WRITE_TIMEOUT = 0 -- Instant timeout

--------------------
-- Convienence functions for reading dynamixel packets
DP1.parse_status_packet = function(pkt) -- 1.0 protocol
   local t = {}
   t.id = pkt:byte(3)
   t.length = pkt:byte(4)
   t.error = pkt:byte(5)
   t.parameter = {pkt:byte(6,t.length+3)}
   t.checksum = pkt:byte(t.length+4)
   return t
end

DP2.parse_status_packet = function(pkt) -- 2.0 protocol
  --print('status pkt',pkt:byte(1,#pkt) )
  local t = {}
  t.id = pkt:byte(5)
  t.length = pkt:byte(6)+2^8*pkt:byte(7)
  t.instruction = pkt:byte(8)
  t.error = pkt:byte(9)
  t.parameter = {pkt:byte(10,t.length+5)}
  t.checksum = string.char( pkt:byte(t.length+6), pkt:byte(t.length+7) );
  return t
end

-- RX (uses 1.0)
-- Format: { Register Address, Register Byte Size}
local rx_registers = {
  ['id'] = {3,1},
  ['baud'] = {4,1},
  ['delay'] = {5,1},
  ['torque_enable'] = {24,1},
  ['led'] = {25,1},
  ['command_position'] = {30,2},
  ['position'] = {36,2},
  ['battery'] = {42,2},
  ['temperature'] = {43,1},
}
libDynamixel.rx_registers = rx_registers

-- MX
-- http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
-- Convention: {string.char( ADDR_LOW_BYTE, ADDR_HIGH_BYTE ), n_bytes_of_value}
local mx_registers = {
  ['model_num'] = {string.char(0,0),2},
  ['firmware'] = {string.char(2,0),1},
  ['id'] = {string.char(3,0),1},
  ['baud'] = {string.char(4,0),1},
  ['delay'] = {string.char(5,0),1},
  ['max_torque'] = {string.char(14,0),2},
  ['status_return_level'] = {string.char(16,0),1},
  ['torque_enable'] = {string.char(24,0),1},
  ['led'] = {string.char(25,0),1},
  
  -- Position PID Gains (position control mode)
  ['position_p'] = {string.char(28,0),1},
  ['position_i'] = {string.char(27,0),1},
  ['position_d'] = {string.char(26,0),1},
  
  ['command_position'] = {string.char(30,0),2},
  ['velocity'] = {string.char(32,0),2},
  ['position'] = {string.char(36,0),2},
  ['speed'] = {string.char(38,0),2},
  ['load'] = {string.char(40,0),2},
  
  ['battery'] = {string.char(42,0),2},
  ['temperature'] = {string.char(43,0),1},
}
libDynamixel.mx_registers = mx_registers

-- Dynamixel PRO
-- English to Hex Addresses of various commands/information
-- Convention: string.char( LOW_BYTE, HIGH_BYTE )
-- http://support.robotis.com/en/product/dynamixel_pro/control_table.htm
local nx_registers = {
  
  -- New API --
  -- ENTER EEPROM AREA

  -- General Operation information
  ['model_num']  = {string.char(0x00,0x00),2},
  ['model_info'] = {string.char(0x02,0x00),4},
  ['firmware'] =   {string.char(0x06,0x00),1},
  ['id'] =   {string.char(0x07,0x00),1},
  -- Baud
  --[[
  0: 2400 ,1: 57600, 2: 115200, 3: 1Mbps, 4: 2Mbps
  5: 3Mbps, 6: 4Mbps, 7: 4.5Mbps, 8: 10.5Mbps
  --]]
  ['baud'] = {string.char(0x08,0x00),1},
  -- Delay in us: wish to have zero
  ['delay'] = {string.char(9,0),1},
  
  -- Operation Mode
  -- Mode 0: Torque Control
  -- Mode 1: Velocity Control
  -- Mode 2: Position Control
  -- Mode 3: position-Velocity Control
  ['mode'] = {string.char(0x0B,0x00),1},
  ['homing_offset'] = {string.char(13,0x00),4},
  
  -- Limits
  ['max_temperature'] = {string.char(0x15,0x00,1)},
  ['max_voltage'] = {string.char(0x16,0x00),2},
  ['min_voltage'] = {string.char(0x18,0x00),2},
  ['max_acceleration'] = {string.char(0x1A,0x00),4},
  ['max_torque'] = {string.char(0x1E,0x00),2},
  ['max_velocity'] = {string.char(0x20,0x00),4},
  ['max_position'] = {string.char(0x24,0x00),4},
  ['min_position'] = {string.char(0x28,0x00),4},
  ['shutdown'] = {string.char(0x30,0x00),1},
  
  -- ENTER RAM AREA
  ['torque_enable'] = {string.char(0x32,0x02),1},
  -- Position Options --
  -- Position Commands (position control mode)
  ['command_position'] = {string.char(0x54,0x02),4},
  ['command_velocity'] = {string.char(0x58,0x02),4},
  ['command_acceleration'] = {string.char(0x5E,0x02),4},
  -- Position PID Gains (position control mode)
  ['position_p'] = {string.char(0x52,0x02),2},
  ['position_i'] = {string.char(0x50,0x02),2},
  ['position_d'] = {string.char(0x4E,0x02),2},
  -- Velocity PID Gains (position control mode)
  ['velocity_p'] = {string.char(0x46,0x02),2},
  ['velocity_i'] = {string.char(0x4A,0x02),2},
  ['velocity_d'] = {string.char(0x4C,0x02),2},
  
  -- Low Pass Fitler settings
  ['position_lpf'] = {string.char(0x42,0x02),4},
  ['velocity_lpf'] = {string.char(0x46,0x02),4},
  -- Feed Forward mechanism
  ['acceleration_ff'] = {string.char(0x3A,0x02),4},
  ['velocity_ff'] = {string.char(0x3E,0x02),4},
  
  -- Torque options --
  -- Commanded Torque (torque control mode)
  ['command_torque'] = {string.char(0x5C,0x02),4},
  -- Current (V=iR) PI Gains (torque control mode)
  ['current_p'] = {string.char(0x38,0x02),2},
  ['current_i'] = {string.char(0x36,0x02),2},

  -- LED lighting
  ['led_red'] = {string.char(0x33,0x02),1},
  ['led_green'] = {string.char(0x34,0x02),1},
  ['led_blue'] = {string.char(0x35,0x02),1},
  
  -- Present information
  ['position'] = {string.char(0x63,0x02),4},
  ['velocity'] = {string.char(0x67,0x02),4},
  ['current'] = {string.char(0x6D,0x02),2},
  ['load'] = {string.char(0x6B,0x02),2},
  ['voltage'] = {string.char(0x6F,0x02),2},
  ['temperature'] = {string.char(0x71,0x02),1},

  -- External Data
  ['data1'] = {string.char(0x72,0x02),2},
  ['data2'] = {string.char(0x74,0x02),2},
  ['data3'] = {string.char(0x76,0x02),2},
  ['data4'] = {string.char(0x78,0x02),2},
  --
  ['data']  = {string.char(0x72,0x02),8},
  
  -- Status return
  ['status_return_level'] = {string.char(0x7B,0x03),1},
}
libDynamixel.nx_registers = nx_registers

--------------------
-- Convienence functions for constructing Sync Write instructions
local function sync_write_byte(ids, addr, data)
  local all_data = nil
  local nid = #ids
  -- All get the same value
  if type(data)=='number' then 
    all_data = data
  else
    assert(nid==#data,'Incongruent ids and data')
  end
  
  local t = {}
  local n = 1
  local len = 1 -- byte
  for i = 1,nid do
    t[n] = ids[i]
    t[n+1] = all_data or data[i]
    n = n + len + 1
  end
  
  return t
end

local function sync_write_word(ids, addr, data)
  local all_data = nil
  local nid = #ids
  if type(data)=='number' then
    -- All get the same value
    all_data = data
  else
    assert(nid==#data,'Incongruent ids and data')
  end

  local t = {}
  local n = 1
  local len = 2 -- word
  for i = 1,nid do
    t[n] = ids[i];
    local val = all_data or data[i]
    -- Word to byte is the same for both packet types...
    t[n+1],t[n+2] = DP2.word_to_byte(val)
    n = n + len + 1;
  end
  return t
end

local function sync_write_dword(ids, addr, data)
  local all_data = nil
  local nid = #ids
  local len = 4
  if type(data)=='number' then
    -- All get the same value
  all_data = data
  else
    assert(nid==#data,'Incongruent ids and data')
  end
  local t = {};
  local n = 1;
  for i = 1,nid do
    t[n] = ids[i];
    local val = all_data or data[i]
    t[n+1],t[n+2],t[n+3],t[n+4] = DP2.dword_to_byte(val)
    n = n + len + 1;
  end
  return t
end

--------------------
-- Initialize functions for reading/writing to NX motors
local nx_single_write = {}
nx_single_write[1] = DP2.write_byte
nx_single_write[2] = DP2.write_word
nx_single_write[4] = DP2.write_dword

local mx_single_write = {}
mx_single_write[1] = DP2.write_byte
mx_single_write[2] = DP2.write_word
mx_single_write[4] = DP2.write_dword

local rx_single_write = {}
rx_single_write[1] = DP1.write_byte
rx_single_write[2] = DP1.write_word
rx_single_write[4] = DP1.write_dword

local sync_write = {}
sync_write[1] = sync_write_byte
sync_write[2] = sync_write_word
sync_write[4] = sync_write_dword

local byte_to_number = {}
byte_to_number[1] = function(byte)
  return byte
end
byte_to_number[2] = DP2.byte_to_word
byte_to_number[4] = DP2.byte_to_dword
libDynamixel.byte_to_number = byte_to_number

-- Old get status method
local function get_status( fd, npkt, protocol, timeout )
  -- TODO: Is this the best default timeout for the new PRO series?
  timeout = timeout or READ_TIMEOUT
  npkt = npkt or 1

  local DP = DP2
  if protocol==1 then DP = DP1 end

  local t0 = unix.time()
  local status_str = ''
  local pkt_cnt = 0
  local statuses = {}
  while unix.time()-t0<timeout do
    local s = unix.read(fd)
    if s then
      local pkts,status_str = DP.input(status_str..s)
      --print('Status sz',#status_str,#pkts)
      if pkts then
        for p,pkt in ipairs(pkts) do
          local status = DP.parse_status_packet( pkt )
          if npkt==1 then return status end
          table.insert( statuses, status )
        end
        if #statuses==npkt then return statuses end
      end -- if pkts
    end
    unix.select({fd},0.001)
  end
  -- Did we timeout?
  return nil
end

--------------------
-- Set NX functions: returns the command to send on the chain
for k,v in pairs( nx_registers ) do
  libDynamixel['set_nx_'..k] = function( motor_ids, values, bus)
    local addr = v[1]
    local sz = v[2]
    
    -- Construct the instruction (single or sync)
    local single = type(motor_ids)=='number'
    local instruction = nil
    if single then
      instruction = nx_single_write[sz](motor_ids, addr, values)
    else
--      print('sync writing')
      local msg = sync_write[sz](motor_ids, addr, values)
      instruction = DP2.sync_write(addr, sz, string.char(unpack(msg)))
    end
    
      if not bus then return instruction end

      -- Clear the reading
      local clr = unix.read(bus.fd)

      -- Write the instruction to the bus 
      stty.flush(bus.fd)
      local ret = unix.write(bus.fd, instruction)
      
      -- Grab any status returns
      if using_status_return and single then
        return get_status( bus.fd, 1 )
      end
    
  end --function
end

--------------------
-- Get NX functions
for k,v in pairs( nx_registers ) do
  local addr = v[1]
  local sz   = v[2]
  libDynamixel['get_nx_'..k] = function( motor_ids, bus )
    -- Construct the instruction (single or sync)
    local instruction
    local nids = 1
    if type(motor_ids)=='number' then
      -- Single motor
      instruction = DP2.read_data(motor_ids, addr, sz)
    else
      instruction = DP2.sync_read(string.char(unpack(motor_ids)), addr, sz)
      nids = #motor_ids
    end
    -- If no pre-existing bus is specified, just return the instruction
    if not bus then return instruction end
    -- Clear old status packets
    repeat buf = unix.read(bus.fd) until not buf
    -- Write the instruction to the bus 
    stty.flush(bus.fd)
    local ret = unix.write(bus.fd, instruction)
    -- Grab the status of the register
    return get_status( bus.fd, nids )
  end --function
end

--------------------
-- Set MX functions
for k,v in pairs( mx_registers ) do
  libDynamixel['set_mx_'..k] = function( motor_ids, values, bus )
    local addr = v[1]
    local sz = v[2]
    
    -- Construct the instruction (single or sync)
    local single = type(motor_ids)=='number'
    local instruction = nil
    if single then
      instruction = mx_single_write[sz](motor_ids, addr, values)
    else
      local msg = sync_write[sz](motor_ids, addr, values)
      instruction = DP2.sync_write(addr, sz, string.char(unpack(msg)))
    end
    
    if not bus then return instruction end

    -- Write the instruction to the bus
    stty.flush(bus.fd)
    local ret = unix.write(bus.fd, instruction)
    
    -- Grab any status returns
    if using_status_return and single then
      local status = get_status( bus.fd )
      if not status then return end
      local value = byte_to_number[sz]( unpack(status.parameter) )
      return status, value
    end
    
  end --function
end

--------------------
-- Get MX functions
for k,v in pairs( mx_registers ) do
  libDynamixel['get_mx_'..k] = function( motor_ids, bus )
    local addr = v[1]
    local sz = v[2]
    
    -- Construct the instruction (single or sync)
    local instruction
    local nids = 1
    if type(motor_ids)=='number' then
      -- Single motor
      instruction = DP2.read_data(motor_ids, addr, sz)
    else
      instruction = DP2.sync_read(string.char(unpack(motor_ids)), addr, sz)
      nids = #motor_ids
    end
    
    if not bus then return instruction end
    
    -- Clear old status packets
    local clear = unix.read( bus.fd )
    
    -- Write the instruction to the bus 
    stty.flush(bus.fd)
    local ret = unix.write( bus.fd, instruction)
    
    -- Grab the status of the register
    local status = get_status( bus.fd, nids )
    if not status then return end
    
    local values = {}
    for i,s in ipairs(status) do
      table.insert(values,byte_to_number[sz]( unpack(s.parameter) ))
    end
    return status, values
    
  end --function
end

--------------------
-- Set RX functions
for k,v in pairs( rx_registers ) do
  libDynamixel['set_rx_'..k] = function( motor_ids, values, bus)
    local addr = v[1]
    local sz = v[2]
    
    -- Construct the instruction (single or sync)
    local single = type(motor_ids)=='number'
    local instruction = nil
    if single then
      instruction = rx_single_write[sz](motor_ids, addr, values)
    else
      local msg = sync_write[sz](motor_ids, addr, values)
      instruction = DP1.sync_write(addr, sz, string.char(unpack(msg)))
    end
    
    if not bus then return instruction end

    -- Write the instruction to the bus
    stty.flush(bus.fd)
    local ret = unix.write(bus.fd, instruction)
    
    -- Grab any status returns
    if using_status_return and single then
      local status = get_status( bus.fd, 1 )
      return status[1]
    end
    
  end --function
end

--------------------
-- Get RX functions
for k,v in pairs( rx_registers ) do
  libDynamixel['get_rx_'..k] = function( motor_ids, bus )
    local addr = v[1]
    local sz = v[2]
    
    -- Construct the instruction (single or sync)
    local instruction = nil
    -- Single motor
    instruction = DP1.read_data(motor_id, addr, sz)

    if not bus then return instruction end
    
    -- Clear old status packets
    local clear = unix.read(bus.fd)
    
    -- Write the instruction to the bus 
    stty.flush(bus.fd)
    local ret = unix.write(bus.fd, instruction)
    
    -- Grab the status of the register
    local status = get_status( bus.fd, 1, 1 )
    local value = byte_to_number[sz]( unpack(status[1].parameter) )
    return status, value
    
  end --function
end

--------------------
-- Ping functions
libDynamixel.send_ping = function( id, protocol, bus, twait )
  protocol = protocol or 2
  local instruction = nil
  if protocol==1 then
    instruction = DP1.ping(id)
  else
    instruction = DP2.ping(id)
  end
  if not bus then return instruction end

  stty.flush(bus.fd)
  local ret    = unix.write(bus.fd, instruction)
  local status = get_status( bus.fd, 1, protocol, twait )
  if status then return status end
end

local function ping_probe(self, protocol, twait)
  local found_ids = {}
  protocol = protocol or 2
  twait = twait or READ_TIMEOUT
  for id = 0,253 do
    local status = 
      libDynamixel.send_ping( id, protocol, self, twait )
    if status then
      print( string.format('Found %d.0 Motor: %d\n',protocol,status.id) )
      table.insert( found_ids, status.id )
    end
    -- Wait .1 ms
    unix.usleep(1e4)
  end
  return found_ids
end

--------------------
-- Generator of a new bus
function libDynamixel.new_bus( ttyname, ttybaud )
  -------------------------------
  -- Find the device
  local baud = ttybaud or 3000000;
  if not ttyname then
    local ttys = unix.readdir("/dev");
    for i=1,#ttys do
      if ttys[i]:find("tty.usb") or ttys[i]:find("ttyUSB") then
        ttyname = "/dev/"..ttys[i]
        -- TODO: Test if in use
        break
      end
    end
  end
  assert(ttyname, "Dynamixel tty not found");
  -------------------------------

  -------------------
  -- Setup serial port
  local fd = unix.open(ttyname, unix.O_RDWR+unix.O_NOCTTY+unix.O_NONBLOCK);
  assert(fd > 2, string.format("Could not open port %s, (%d)", ttyname, fd) );
  stty.raw(fd)
  stty.serial(fd)
  stty.speed(fd, baud)
  -------------------

  -------------------
  -- Object of the Dynamixel
  local obj = {}
  obj.fd = fd
  obj.ttyname = ttyname
  obj.baud = baud
  -- Close out the device
  obj.close = function (self) return unix.close( self.fd )==0 end
  -- Reset the device
  obj.reset = function(self)
    self:close()
    unix.usleep( 1e3 )
    self.fd = libDynamixel.open( self.ttyname )
  end
  obj.ping_probe = ping_probe
  -------------------
  
  -------------------
  -- Add libDynamixel functions
  --[[
  for name,func in pairs( libDynamixel ) do
    obj[name] = func
  end
  -- new_bus not allowed on a current bus
  obj.new_bus = nil
  obj.service = nil
  --]]
  -------------------
  
  -------------------
  -- Read/write properties
  obj.t_read = 0
  obj.t_command = 0
  obj.t_diff = 0
  obj.commands = {}
  obj.requests = {}
  obj.is_syncing = true
  obj.nx_on_bus = nil -- ids of nx
  obj.mx_on_bus = nil -- ids of mx
  obj.ids_on_bus = nil --all on the bus
  obj.name = 'Default'
  obj.message = nil
  obj.is_reading = false
  -------------------
  
  return obj
end

---------------------------
-- Service multiple Dynamixel buses
libDynamixel.service = function( dynamixels, main )
  
  -- Enable the main function as a coroutine thread
  local main_thread = nil
  if main then
    main_thread = coroutine.create( main )
  end

  -- Start the streaming of each dynamixel
  -- Instantiate the dynamixel coroutine thread
  local dynamixel_fds = {}
  local fd_to_dynamixel = {}
  for i,dynamixel in ipairs(dynamixels) do
    -- Set up easy access to select IDs
    table.insert(dynamixel_fds,dynamixel.fd)
    fd_to_dynamixel[dynamixel.fd] = dynamixel
    dynamixel.thread = coroutine.create(
    function(has_data,t)
      -- time is the parameter
      -- Make a read all NX command
      local DP = DP2
      
      local did_command = false
      local did_request = false
      -- The coroutine should never end
      local fd = dynamixel.fd
      -- Update the time after every yield
      while true do -- read/write loop

        -- See what we got
        has_data, t = coroutine.yield()

        --------------------
        -- Sync write an command in the queue
        dynamixel.is_reading = false
        local n_cmd_bytes = 0
        local n_cmd_left = #dynamixel.commands
        did_command = false

        while n_cmd_left>0 do
          -- Pop a command
          local command = table.remove(dynamixel.commands,1)
          n_cmd_left = n_cmd_left - 1
          -- flush out old things in the buffer
          local tt0 = unix.time()
          local flush_ret = stty.flush(fd)
          -- write the new command
          local cmd_ret = unix.write( fd, command )
          -- possibly need a drain? Robotis does not
          local flush_ret = stty.drain(fd)
          local tt1 = unix.time()
          
          assert(#command==cmd_ret,
            string.format('BAD INST WRITE: %s',dynamixel.name))
          n_cmd_bytes = n_cmd_bytes+cmd_ret
          -- What if there was data on the bus... that is in the non-sync read
          -- It would be an error status
          dynamixel.t_diff = t-dynamixel.t_command

--          print(string.format('tts: %.2f t_diff: %.2f',(tt1-tt0)*1000, dynamixel.t_diff*1000))
          dynamixel.t_command = t
          did_command = true
          -- sleep a little
          if n_cmd_left>0 then unix.usleep(1e3) end
          --print('wrote',cmd_ret)
        end -- If sent command
        
        --------------------
        -- Request data from the chain
        did_request = false
        local n_req_left = #dynamixel.requests
        while n_req_left>0 do
          dynamixel.is_reading = true
          -- Non-block saving a leftovers (if any)
          local leftovers = unix.read(fd)
          assert(leftovers~=-1, 'BAD Clearing READ')
          -- Pop a request
          local request = table.remove(dynamixel.requests,1)
          n_req_left = n_req_left - 1
          -- Write the read request command to the chain
          local inst = request.inst
          stty.flush(fd)
          local req_ret = unix.write(fd, inst)
          -- If -1 returned, the bus may be detached - throw an error
          assert(req_ret==#inst,
            string.format('BAD READ REQ on %s',dynamixel.name))
          -- Set a timeout for this request
          dynamixel.timeout = t + READ_TIMEOUT
          -- Yield for others on their FDs
          
          -- Accrue values from the status packets
          local values = {}
          local pkts = {}
          local nids = request.nids
          local n_recv = 0
          --print('Expecting',nids)
          local register =request.reg
          status_str = ''
          repeat
            did_request = true
            local new_status_str, t = coroutine.yield()
            assert(new_status_str~=-1,string.format('BAD READ: %s',dynamixel.name))
            -- What is the meaning of a non-read here?
            if not new_status_str then
              --print('READ TIMEOUT',n_recv,nids,dynamixel.name)
              break
            end
            --assert(new_status_str, string.format('NO READ: %s',dynamixel.name))
            -- Process the status string into a packet
            pkts, status_str = DP.input( status_str..new_status_str )
            n_recv = n_recv + #pkts
            --print('npkts',#pkts,n_recv)
            -- For each packet, append to the values table
            for _,pkt in ipairs(pkts) do
              local status = DP.parse_status_packet( pkt )
              local read_parser = byte_to_number[ #status.parameter ]
              -- Check if there is a parser
              --[[
              assert(read_parser, 
              string.format('Status error for %s from %d: %d (%d)',
              register,status.id,status.error,#status.parameter) )
              --]]
              -- Convert the value into a number from bytes if a parser exists!
              if read_parser then
                values[status.id] = read_parser( unpack(status.parameter) )
              else
                values[status.id] = status.parameter
              end
            end
            -- Remember the read time
            dynamixel.t_read = t
            if dynamixel.callback then dynamixel:callback(values,register) end
            -- If done, then do not yield...
            -- Then, we write a command to the FD more quickly
            if n_recv==nids then break end
            -- Yield to the next process
          until false
        end -- if requested data
        dynamixel.is_reading = false

      end -- read/write loop
    end) -- coroutine function
    coroutine.resume(dynamixel.thread,false,unix.time() )
  end
  
  -- While servicing the dynamixel fds
  while #dynamixel_fds>0 do
    --------------------
    -- Perform select on all dynamixels
    local status, ready = unix.select( dynamixel_fds, WRITE_TIMEOUT )
    --unix.usleep(WRITE_TIMEOUT)
    local t = unix.time()

    --util.ptable(ready)
    --print(status)
    --------------------
    -- Loop through the dynamixel chains
    for i_fd,is_ready in pairs(ready) do
    -- Grab the dynamixel chain
    local who_to_service = fd_to_dynamixel[i_fd]
    --for i_fd,who_to_service in pairs(fd_to_dynamixel) do
      
      -- Grab data
      local str = unix.read(i_fd)

      --[[
      local flush_ret = stty.flush(i_fd)
      -- write the new command
      local cmd_ret = unix.write( i_fd, who_to_service.commands )
      -- possibly need a drain? Robotis does not
      local flush_ret = stty.drain(i_fd)
      --]]
      -- Check if the Dynamixel has information available
      if who_to_service.is_reading and (not str) and (t<who_to_service.timeout) then
        --print('waiting...')
        -- do not resume, since we are waiting on the data
      elseif (not who_to_service.is_reading) and str then
        -- We also have nothing to send...
        -- Non-block saving a leftovers (if any)
        assert(unix.read(i_fd)~=-1,'Unplugged?')
      elseif (str or (#who_to_service.commands>0) or (#who_to_service.requests>0)) then
--        print(status,'n_commands, n_requests',#who_to_service.commands,#who_to_service.requests,who_to_service.is_reading)
        --if str then print('read',#str) end
        -- Resume the thread
        local status_code, param = coroutine.resume(who_to_service.thread,str,t)
        if not status_code then
          print( 'Dead dynamixel coroutine!', who_to_service.name, param )
          who_to_service:close()
          who_to_service.message = param
          local i_to_remove = 0
          for i,fd in ipairs(dynamixel_fds) do
            if fd==i_fd then i_to_remove = i end
          end
          table.remove(dynamixel_fds,i_to_remove)
        end
      end -- if resuming
--]]
    end -- pairs(ready)
    
    --------------------
    -- Process the main thread after each coroutine yields
    -- This main loop should update the dynamixel chain commands
    if main_thread then
      local status_code, main_param = coroutine.resume( main_thread )
      if not status_code then 
        print('Dead main coroutine!',main_param)
        main_thread = nil
      end
    end
    
  end -- while servicing
  print'Nothing left to service!'
end

return libDynamixel
