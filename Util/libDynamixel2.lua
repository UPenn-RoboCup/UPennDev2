-- Dynamixel Library
-- (c) 2013 Stephen McGill
-- (c) 2013 Daniel D. Lee

local libDynamixel = {}
local DP1 = require('DynamixelPacket1') -- 1.0 protocol
local DP2 = require('DynamixelPacket2') -- 2.0 protocol
unix = require('unix')
stty = require('stty')
local using_status_return = false

-- RX (uses 1.0)
-- Format: { Register Address, Register Byte Size}
local rx_registers = {
	['id'] = {3,1}
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
	['id'] = {3,1}
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
	local nid = #ids
	local all_data = nil
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
	local nid = #ids;
	local all_data = nil
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
	local nid = #ids;
	local len = 4;
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
		t[n+1],t[n+2],t[n+3],t[n+4] = 
			DP2.dword_to_byte(val)
		n = n + len + 1;
	end
	return t
end

--------------------
-- Convienence functions for reading dynamixel packets
local function parse_status_packet1(pkt) -- 1.0 protocol
   local t = {};
   t.id = pkt:byte(3);
   t.length = pkt:byte(4);
   t.error = pkt:byte(5);
   t.parameter = {pkt:byte(6,t.length+3)};
   t.checksum = pkt:byte(t.length+4);
   return t;
end

local function parse_status_packet2(pkt) -- 2.0 protocol
	local t = {}
	t.id = pkt:byte(5)
	t.length = pkt:byte(6)+2^8*pkt:byte(7)
	t.instruction = pkt:byte(8)
	t.error = pkt:byte(9)
	t.parameter = {pkt:byte(10,t.length+5)}
	t.checksum = string.char( pkt:byte(t.length+6), pkt:byte(t.length+7) );
	return t;
end

-- Old get status method
local function get_status( protocol, fd, npkt, expected_sz, timeout )
	-- TODO: Is this the best default timeout for the new PRO series?
	timeout = timeout or 0.1;

	local parser = parse_status_packet2
	local inp_segmenter = DP2.input
	if protocol==1 then
		parser = parse_status_packet1
	  inp_segmenter = DP1.input
	end

	local t0 = unix.time();
	local str = "";
	local pkt_cnt = 0;
	local statuses = {}
	while unix.time()-t0<timeout do
		local s = unix.read(fd);
		if s then
			str = str..s
			local pkts = inp_segmenter(str)
			if pkts then
				for p,pkt in ipairs(pkts) do
					local status = parser( pkt )
					table.insert( statuses, status )
				end
				if #statuses==npkt then
					return statuses
				end
			end -- if pkts
		end
		-- TODO: yield the sleep amount
		unix.usleep(100);
	end
	-- Did we timeout?
	return nil
end

--------------------
-- Set functions
local nx_single_write = {}
nx_single_write[1] = DP2.write_byte
nx_single_write[2] = DP2.write_word
nx_single_write[4] = DP2.write_dword

local nx_sync_write = {}
nx_sync_write[1] = sync_write_byte
nx_sync_write[2] = sync_write_word
nx_sync_write[4] = sync_write_dword

for k,v in pairs( nx_registers ) do
	libDynamixel['set_nx_'..k] = function( bus, motor_ids, values )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
		-- Use single writes or sync write
		local single = type(motor_ids)=='table'
		
		-- Clear old status packets
		local clear = unix.read(fd)
		
		-- Construct the instruction (single or sync)
		local instruction = nil
		if single then
			instruction = nx_single_write[sz](motor_ids, addr, values)
		else
			local msg = nx_sync_write[sz](motor_ids, addr, values)
			instruction = DP2.sync_write(addr, sz, string.char(unpack(msg)))
		end
		
		-- Write the instruction to the bus 
		local ret = unix.write(fd, instruction)
		
		-- Grab any status returns
		if using_status_return and single then
			-- TODO: Provide an expected size
			local status = libDynamixel.get_status( 2, fd, 1, nil )
		end
		
	end --function
end

--------------------
-- Get functions
for k,v in pairs( nx_registers ) do
	libDynamixel['get_nx_'..k] = function( bus, motor_ids, values )
		local addr = v[1]
		local sz = v[2]
		local fd = bus.fd
		
		-- Use single writes or sync write
		local single = type(motor_ids)=='table'
		
		-- Clear old status packets
		local clear = unix.read(fd)
		
		-- Construct the instruction (single or sync)
		local instruction = nil
		local nids = 1
		if single then
			instruction = DP2.read_data(id, addr, sz);
		else
			instruction = DP2.sync_read(string.char(unpack(motor_ids)), addr, sz)
			nids = #motor_ids
		end
		
		-- Write the instruction to the bus 
		local ret = unix.write(fd, instruction)
		
		-- Grab the status of the register
		-- TODO: Provide an expected size
		local status = libDynamixel.get_status( 2, fd, nids, nil )
		
	end --function
end

-- TODO: Provide expected ping response size
libDynamixel.send_ping = function( self, id, protocol )
	protocol = protocol or 2
	local instruction = nil
	if protocol==1 then
		instruction = DP1.ping(id)
	else
		instruction = DP2.ping(id)
	end
	unix.write(self.fd, instruction);
	return libDynamixel.get_status1(protocol,self.fd,1,nil,twait);
end

libDynamixel.ping_probe = function(self, protocol, twait)
	twait = twait or 0.010
	for id = 0,253 do
		local status = libDynamixel.send_ping( self, id, protocol );
		if status then
			io.write('FOUND 1.0: ',status.id,'\n')
		end
	end
end

function libDynamixel.new_bus( ttyname, ttybaud )
	-------------------------------
	-- Find the device
	local baud = ttybaud or 1000000;
	if not ttyname then
		local ttys = unix.readdir("/dev");
		for i=1,#ttys do
			if (string.find(ttys[i], "tty.usb") or
			string.find(ttys[i], "ttyUSB")) then
				ttyname = "/dev/"..ttys[i];
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
	stty.raw(fd);
	stty.serial(fd);
	stty.speed(fd, baud);
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
		unix.usleep( 1e5 )
		self.fd = libDynamixel.open( self.ttyname )
	end
	-------------------
	
	-------------------
	-- Add libDynamixel functions
	for name,func in pairs( libDynamixel ) do
		obj[name] = func
	end
	-- new_bus not allowed on a current bus
	obj.new_bus = nil
	-------------------
	
	return obj
end

return libDynamixel
