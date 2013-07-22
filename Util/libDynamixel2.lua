-- Dynamixel Library
-- (c) 2013 Stephen McGill
-- (c) 2013 Daniel D. Lee
-- Support: http://support.robotis.com/en/product/dynamixel_pro/communication/instruction_status_packet.htm

local libDynamixel = {}
local DP1 = require'DynamixelPacket1' -- 1.0 protocol
local DP2 = require'DynamixelPacket2' -- 2.0 protocol
local unix = require'unix'
local stty = require'stty'
local using_status_return = false

--------------------
-- Convienence functions for reading dynamixel packets
DP1.parse_status_packet = function(pkt) -- 1.0 protocol
   local t = {}
   t.id = pkt:byte(3)
   t.length = pkt:byte(4)
   t.error = pkt:byte(5)
   t.parameter = {pkt:byte(6,t.length+3)}
   t.checksum = pkt:byte(t.length+4)
   return t;
end

DP2.parse_status_packet = function(pkt) -- 2.0 protocol
	local t = {}
	t.id = pkt:byte(5)
	t.length = pkt:byte(6)+2^8*pkt:byte(7)
	t.instruction = pkt:byte(8)
	t.error = pkt:byte(9)
	t.parameter = {pkt:byte(10,t.length+5)}
	t.checksum = string.char( pkt:byte(t.length+6), pkt:byte(t.length+7) );
	return t;
end

-- RX (uses 1.0)
-- Format: { Register Address, Register Byte Size}
local rx_registers = {
	['id'] = {3,1},
  ['baud'] = {4,1},
	['delay'] = {5,1},
	['torque_enable'] = {24,1},
	['led'] = {25,1},
	['command'] = {30,2},
	['position'] = {36,2},
	['battery'] = {42,2},
	['temperature'] = {43,1},
}

-- MX
local mx_registers = {
	['id'] = {3,1},
  ['baud'] = {4,1},
	['delay'] = {5,1},
	['status_return_level'] = {16,1},
	['torque_enable'] = {24,1},
	['led'] = {25,1},
	
	-- Position PID Gains (position control mode)
	['position_p'] = {28,1},
	['position_i'] = {27,1},
	['position_d'] = {26,1},
	
	['command'] = {30,2},
	['velocity'] = {32,2},
	['position'] = {36,2},
	
	['battery'] = {42,2},
	['temperature'] = {43,1},
}

-- Dynamixel PRO
-- English to Hex Addresses of various commands/information
-- Convention: string.char( LOW_BYTE, HIGH_BYTE )
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
	
	-- Operation Mode
	-- Mode 0: Torque Control
	-- Mode 1: Velocity Control
	-- Mode 2: Position Control
	-- Mode 3: position-Velocity Control
	['mode'] = {string.char(0x0B,0x00),1},
	
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
}

--------------------
-- Convienence functions for constructing Sync Write instructions
local function sync_write_byte(ids, addr, data)
	local nid = 1
	local all_data = #ids
	if type(data)=='number' then
		-- All get the same value
		all_data = data
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
	local nid = #ids
	local len = 4
	local all_data = nil
	if type(data)=='number' then
		-- All get the same value
		all_data = data
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

-- Old get status method
local function get_status( fd, npkt, protocol, timeout )
	-- TODO: Is this the best default timeout for the new PRO series?
	timeout = timeout or 0.05
  npkt = npkt or 1

  local DP = DP2
	if protocol==1 then DP = DP1 end

	local t0 = unix.time()
	local status_str = ''
	local pkt_cnt = 0
	local statuses = {}
	while unix.time()-t0<timeout do
		local s = unix.read(fd);
		if s then
			status_str = status_str..s
			local pkts = DP.input(status_str)
      --print('Status sz',#status_str)
			if pkts then
				for p,pkt in ipairs(pkts) do
					local status = DP.parse_status_packet( pkt )
					table.insert( statuses, status )
				end
				if #statuses==npkt then return statuses end
			end -- if pkts
		end
		-- TODO: yield the sleep amount
		unix.usleep(100)
	end
	-- Did we timeout?
	return nil
end

--------------------
-- Initialize functions for reading/writing to NX motors
local nx_single_write = {}
nx_single_write[1] = DP2.write_byte
nx_single_write[2] = DP2.write_word
nx_single_write[4] = DP2.write_dword

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

--------------------
-- Set NX functions
for k,v in pairs( nx_registers ) do
	libDynamixel['set_nx_'..k] = function( bus, motor_ids, values, serviced )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
    -- TODO: The I/O should be handled elsewhere
		-- Clear old status packets
		local clear = unix.read(fd)
		
		-- Construct the instruction (single or sync)
    local single = type(motor_ids)=='number'
		local instruction = nil
		if single then
			instruction = nx_single_write[sz](motor_ids, addr, values)
		else
			local msg = sync_write[sz](motor_ids, addr, values)
			instruction = DP2.sync_write(addr, sz, string.char(unpack(msg)))
		end
		
    -- TODO: Just queue the instruction on the bus
    -- TODO: Give expected status size in bytes
    -- TODO: How to return data to the sender?
    if serviced then
      return instruction
    end

    -- Write the instruction to the bus 
    local ret = unix.write(fd, instruction)
		
    -- Grab any status returns
    if using_status_return and single then
      local status = get_status( fd, 1 )
      return status[1]
    end
		
	end --function
end

--------------------
-- Get NX functions
for k,v in pairs( nx_registers ) do
	libDynamixel['get_nx_'..k] = function( bus, motor_ids, serviced )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
		-- Construct the instruction (single or sync)
		local instruction = nil
		local nids = 1
		if type(motor_ids)=='table' then
			instruction = DP2.sync_read(string.char(unpack(motor_ids)), addr, sz)
			nids = #motor_ids
		else
      -- Single motor
			instruction = DP2.read_data(motor_ids, addr, sz)
		end
		
    -- TODO: Just queue the instruction on the bus
    -- TODO: Give expected status size in btyes
    -- TODO: How to return data to the sender?
    if serviced then
      local status_sz = 11 + sz*nids
      return instruction, status_sz
    end
    
		-- Clear old status packets
		local clear = unix.read(fd)
    
    -- Write the instruction to the bus 
    local ret = unix.write(fd, instruction)
		
    -- Grab the status of the register
    return get_status( fd, nids )
		
	end --function
end

--------------------
-- Set RX functions
for k,v in pairs( rx_registers ) do
	libDynamixel['set_rx_'..k] = function( bus, motor_ids, values, serviced )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
    -- TODO: The I/O should be handled elsewhere
		-- Clear old status packets
		local clear = unix.read(fd)
		
		-- Construct the instruction (single or sync)
    local single = type(motor_ids)=='number' 
		local instruction = nil
		if single then
			instruction = rx_single_write[sz](motor_ids, addr, values)
		else
			local msg = sync_write[sz](motor_ids, addr, values)
			instruction = DP1.sync_write(addr, sz, string.char(unpack(msg)))
		end
		
    -- TODO: Just queue the instruction on the bus
    -- TODO: Give expected status size in bytes
    -- TODO: How to return data to the sender?
    if serviced then
      return instruction
    end

    -- Write the instruction to the bus 
    local ret = unix.write(fd, instruction)
		
    -- Grab any status returns
    if using_status_return and single then
      local status = get_status( fd, 1 )
      return status[1]
    end
		
	end --function
end

--------------------
-- Get RX functions
for k,v in pairs( rx_registers ) do
	libDynamixel['get_rx_'..k] = function( bus, motor_id, serviced )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
		-- Construct the instruction (single or sync)
		local instruction = nil
    -- Single motor
		instruction = DP1.read_data(motor_id, addr, sz)
		
    -- TODO: Just queue the instruction on the bus
    -- TODO: Give expected status size in btyes
    -- TODO: How to return data to the sender?
    if serviced then
      -- Return the instruction and expected status packet size
      return instruction, 6 + sz
    end
    
		-- Clear old status packets
		local clear = unix.read(fd)
    
    -- Write the instruction to the bus 
    local ret = unix.write(fd, instruction)
		
    -- Grab the status of the register
    local status = get_status( fd, 1, 1 )
    local value = byte_to_number[sz]( unpack(status[1].parameter) )
    return status, value
		
	end --function
end

--------------------
-- Ping functions
libDynamixel.send_ping = function( self, id, protocol, twait )
	protocol = protocol or 2
	local instruction = nil
	if protocol==1 then
		instruction = DP1.ping(id)
	else
		instruction = DP2.ping(id)
	end
  local fd = self.fd
	unix.write(self.fd, instruction)
  local status = get_status( fd, protocol, 1, twait )
  if status then return status[1] end
end

libDynamixel.ping_probe = function(self, protocol, twait)
  protocol = protocol or 2
	twait = twait or 0.010
  local found_ids = {}
	for id = 0,253 do
		local status = libDynamixel.send_ping( self, id, protocol, twait )
		if status then
      print( string.format('Found %d.0 Motor: %d\n',protocol,status.id) )
      table.insert( found_ids, status.id )
		end
    unix.sleep( twait )
	end
  return found_ids
end

--------------------
-- Generator of a new bus
function libDynamixel.new_bus( ttyname, ttybaud )
	-------------------------------
	-- Find the device
	local baud = ttybaud or 1000000;
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
	obj.close = function (self)
		return unix.close( self.fd )==0
	end
	-- Reset the device
	obj.reset = function(self)
		self:close()
		unix.usleep( 1e3 )
		self.fd = libDynamixel.open( self.ttyname )
	end
  obj.t_last = unix.time()
	-------------------
	
	-------------------
	-- Add libDynamixel functions
	for name,func in pairs( libDynamixel ) do
		obj[name] = func
	end
	-- new_bus not allowed on a current bus
	obj.new_bus = nil
	obj.service = nil
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
  local fd_to_dynamixel_id = {}
	for i,dynamixel in ipairs(dynamixels) do
    -- Set up easy access to select IDs
    fd_to_dynamixel[dynamixel.fd] = dynamixel
    fd_to_dynamixel_id[dynamixel.fd] = i
    table.insert(dynamixel_fds,dynamixel.fd)
    -- Check which ids are on this chain
    io.write('Checking ids on ',dynamixel.name,' chain...')
    io.flush()
		dynamixel.ids_on_bus = dynamixel:ping_probe()
    io.write'Done!\n'
    io.flush()
		dynamixel.t_last = unix.time()
		dynamixel.thread = coroutine.create( 
		function()
      
      -- Make a read-all command
      --[[
      local sync_read_all_cmd, sync_read_all_status_sz = 
        dynamixel:get_nx_position( dynamixel.ids_on_bus, true )
      -- TODO: Make read commands for each individual joint
      -]]
      local single_read_cmd, single_read_sz = 
        dynamixel:get_rx_position( 6, true )
      
      -- The coroutine should never end
      local fd = dynamixel.fd
      local led_state = 1
      local time_elapsed = unix.time()
			while true do -- read/write loop
        
        -- Ask for data on the chain
        -- TODO: Clear the bus with a unix.read()?
        local read_ret = unix.write(fd, single_read_cmd)
        assert(read_ret~=-1,string.format('BAD READ REQ on %s',dynamixel.name))
		
        -- Grab the status return from the bus
        local status_str = ''
        while true do -- Status packet buffer fill
          -- Wait for the buffer to fill
          coroutine.yield()
          -- Read from the buffer
          local status_buf = unix.read( fd, single_read_sz-#status_str )
          -- If no return, something maybe went awry with the dynamixel
          assert(status_buf~=-1,string.format('BAD READ on %s',dynamixel.name))
          --print('Read',status_buf,'bytes')
          -- Append it to the status string
          if status_buf then
            status_str = status_str..status_buf
            if #status_str>=single_read_sz then break end
          end
        end -- Status packet buffer fill
        
        -- Yield the status string returned from the chain
        dynamixel.t_last = unix.time()
        -- TODO: queue the coroutine to be resumed, independent of select
        local status = DP1.parse_status_packet(status_str)
        local value = byte_to_number[2]( unpack(status.parameter) )
        coroutine.yield( value )
        

        -- Write data to the chain
        local t_diff_elapsed = unix.time()-time_elapsed
        if t_diff_elapsed>1 then
          local single_led_cmd, single_led_sz = 
            dynamixel:set_rx_led( 6, led_state, true )
          local ret = unix.write(fd, single_led_cmd)
          assert(ret~=-1,string.format('BAD WRITE on %s',dynamixel.name))
          led_state = 1-led_state
          time_elapsed = unix.time()
        end
        
        
        -- TODO: Deal with status return packet or not...

      end -- read/write loop

		end -- coroutine function
		)
	end
  
  -- Initialize them
  for i,dynamixel in ipairs(dynamixels) do
    print('initializing thread for',dynamixel.name)
    local status_code, param = coroutine.resume( dynamixel.thread )
  end
  
  -- Loop and sleep appropriately
	while #dynamixels>0 do
    
    -- Begin to re-enqueue commands on the chains
    -- TODO: Use the dynamixel objects somehow to accept more commands...
    -- TODO: Can you select on the finish of write?
    -- TODO: Change from positio reading to other reading?
    --[[
    for i,is_commanded in pairs(command_queue) do
      local status_code, param = coroutine.resume( who_to_service.thread )
    end
    --]]

    -- Perform Select on all dynamixels
    local status_timeout = 1 -- 1 second timeout
    local status, ready = unix.select( dynamixel_fds, status_timeout )
    --print('Post select',status, ready)
    for i,is_ready in pairs(ready) do
      if is_ready then
        -- Grab the dynamixel
        local who_to_service = fd_to_dynamixel[i]
        -- Resume the thread
        local status_code, param = coroutine.resume( who_to_service.thread )
        -- Check if there were errors in the coroutine
        if not status_code then
          print( 'Dead dynamixel coroutine!', who_to_service.name, param )
          who_to_service:close()
          local d_id = fd_to_dynamixel_id[i]
          table.remove(dynamixels,d_id)
          table.remove(dynamixel_fds,d_id)
        elseif param and who_to_service.callback then
          -- Process the callback
          -- TODO: Put this into shared memory via callback
          who_to_service.callback( param )
          local status_code, param = coroutine.resume( who_to_service.thread )
          --print('Post callback resume',status_code, param)
        end
      end
    end
    
    -- Process the main thread
    if main_thread then
      coroutine.resume( main_thread )
    end
	end -- while servicing
  print'Nothing left to service!'
end

return libDynamixel
