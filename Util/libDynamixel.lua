-- Dynamixel Library
-- (c) 2013 Stephen McGill
-- (c) 2013 Daniel D. Lee
-- Support: http://support.robotis.com/en/product/dynamixel_pro/communication/instruction_status_packet.htm

local libDynamixel = {}
local lD = libDynamixel
local DP1 = require'DynamixelPacket1' -- 1.0 protocol
local DP2 = require'DynamixelPacket2' -- 2.0 protocol
libDynamixel.DP2 = DP2
local unix = require'unix'
local stty = require'stty'

-- OSX is bad with the FTDI :/
local READ_TIMEOUT = 1 / 30

--local READ_TIMEOUT = 1 / 60
--local READ_TIMEOUT = 1 / 80

--local READ_TIMEOUT = 1 / 90

local using_status_return = true
local using_status_return_mx = false

-- Cache
local char = string.char

--------------------
-- Convienence functions for reading dynamixel packets
function DP1.parse_status_packet(pkt) -- 1.0 protocol
	local len = pkt:byte(4)
	return {
		id = pkt:byte(3),
		length = len,
		error = pkt:byte(5),
		parameter = {pkt:byte(6, len + 3)},
		checksum = pkt:byte(len + 4),
	}
end

function DP2.parse_status_packet(pkt) -- 2.0 protocol
	--print('status pkt',pkt:byte(1,#pkt) )
	local err, len = pkt:byte(9), pkt:byte(6) + 256 * pkt:byte(7)
	--[[
	if err>0 then
	return {
	id = pkt:byte(5),
	length = len,
	instruction = pkt:byte(8),
	error = err,
	checksum = char(pkt:byte(10), pkt:byte(11))
	}
	else
	--]]
	return {
		id = pkt:byte(5),
		length = len,
		instruction = pkt:byte(8),
		error = err,
		parameter = {pkt:byte(10, len+5)},
		raw_parameter = pkt:sub(10, len+5),
		checksum = char(pkt:byte(len+6), pkt:byte(len+7))
	}
	--end
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
	['command'] = {30,2},
	['position'] = {36,2},
	['battery'] = {42,2},
	['temperature'] = {43,1},
	torque_mode = {70,1},
	command_torque = {71,2},
	command_acceleration = {73,1},
	everything = {36,8},
}
libDynamixel.rx_registers = rx_registers

libDynamixel.model_number_lookup = {
	[-14336] = 'H42-20-S300-R',
	[1] = 'H54-100-S500-R',
	[1] = 'H54-200-S500-R',
	[29] = 'MX-28R'
}

-- MX
-- http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
-- Convention: {char( ADDR_LOW_BYTE, ADDR_HIGH_BYTE ), n_bytes_of_value}
local mx_registers = {
	['model_number'] = {char(0,0),2},
	['firmware'] = {char(2,0),1},
	['id'] = {char(3,0),1},
	['baud'] = {char(4,0),1},
	['delay'] = {char(5,0),1},
	['return_delay_time'] = {char(5,0),1},
	['min_voltage'] = {char(12,0),1},
	['max_voltage'] = {char(13,0),1},
	['max_torque'] = {char(14,0),2},
	['status_return_level'] = {char(16,0),1},
	['alarm_led'] = {char(17,0),1},
	['alarm_shutdown'] = {char(18,0),1},

	['torque_enable'] = {char(24,0),1},
	['led'] = {char(25,0),1},

	-- Position PID Gains (position control mode)
	['position_p'] = {char(28,0),1},
	['position_i'] = {char(27,0),1},
	['position_d'] = {char(26,0),1},
	-- Commanded
	['command_position'] = {char(30,0),2},
	['velocity'] = {char(32,0),2},
	-- Sensed
	['position'] = {char(36,0),2},
	['speed'] = {char(38,0),2},
	['load'] = {char(40,0),2},
	['voltage'] = {char(42,0),1},
	['temperature'] = {char(43,0),1},

	-- For the MX-106
	current = {char(68,0),2},
	torque_mode = {char(70,0),1},
	mode = {char(70,0),1},
	command_torque = {char(71,0),2},
	command_acceleration = {char(73,0),1},
}
libDynamixel.mx_registers = mx_registers

-- Dynamixel PRO
-- English to Hex Addresses of various commands/information
-- Convention: char( LOW_BYTE, HIGH_BYTE )
-- http://support.robotis.com/en/product/dynamixel_pro/control_table.htm
local nx_registers = {

	-- New API --
	-- ENTER EEPROM AREA

	-- General Operation information
	['model_number']  = {char(0x00,0x00),2},
	['model_information'] = {char(0x02,0x00),4},
	['firmware'] =   {char(0x06,0x00),1},
	['id'] =   {char(0x07,0x00),1},
	-- Baud
	--[[
	0: 2400 ,1: 57600, 2: 115200, 3: 1Mbps, 4: 2Mbps
	5: 3Mbps, 6: 4Mbps, 7: 4.5Mbps, 8: 10.5Mbps
	--]]
	['baud_rate'] = {char(0x08,0x00),1},
	-- Delay in us: wish to have zero
	['return_delay_time'] = {char(9,0),1},

	-- Operation Mode
	-- Mode 0: Torque Control
	-- Mode 1: Velocity Control
	-- Mode 2: Position Control
	-- Mode 3: position-Velocity Control
	['mode'] = {char(0x0B,0x00),1},
	['homing_offset'] = {char(13,0x00),4},

	-- Limits
	['max_temperature'] = {char(0x15,0x00,1)},
	['max_voltage'] = {char(0x16,0x00),2},
	['min_voltage'] = {char(0x18,0x00),2},
	['max_acceleration'] = {char(0x1A,0x00),4},
	['max_torque'] = {char(0x1E,0x00),2},
	['max_velocity'] = {char(0x20,0x00),4},
	['max_position'] = {char(0x24,0x00),4},
	['min_position'] = {char(0x28,0x00),4},
	--
	['data1_mode'] = {char(44,0x00),1},
	['data2_mode'] = {char(45,0x00),1},
	['data3_mode'] = {char(46,0x00),1},
	['data4_mode'] = {char(47,0x00),1},
	--
	['shutdown'] = {char(48,0x00),1},
	['alarm_shutdown'] = {char(48,0x00),1},

	-- ENTER RAM AREA
	['torque_enable'] = {char(0x32,0x02),1},
	-- LED lighting
	['led_red'] = {char(0x33,0x02),1},
	['led_green'] = {char(0x34,0x02),1},
	['led_blue'] = {char(0x35,0x02),1},
	-- Velocity PID Gains (position control mode)
	['velocity_i'] = {char(0x4A,0x02),2},
	['velocity_p'] = {char(0x46,0x02),2},
	--['velocity_d'] = {char(0x4C,0x02),2},
	-- Position PID Gains (position control mode)
	['position_p'] = {char(0x52,0x02),2},
	['position_i'] = {char(0x50,0x02),2},
	['position_d'] = {char(0x4E,0x02),2},

	-- Position Commands (position control mode)
	['command_position'] = {char(0x54,0x02),4},
	['command_velocity'] = {char(0x58,0x02),4},
	['command_acceleration'] = {char(0x5E,0x02),4},

	-- Low Pass Fitler settings
	['position_lpf'] = {char(0x42,0x02),4},
	['velocity_lpf'] = {char(0x46,0x02),4},
	-- Feed Forward mechanism
	['acceleration_ff'] = {char(0x3A,0x02),4},
	['velocity_ff'] = {char(0x3E,0x02),4},

	-- Torque options --
	-- Commanded Torque (torque control mode)
	['command_torque'] = {char(0x5C,0x02),4},
	-- Current (V=iR) PI Gains (torque control mode)
	['current_p'] = {char(0x38,0x02),2},
	['current_i'] = {char(0x36,0x02),2},


	-- Present information
	['position'] = {char(0x63,0x02),4},
	['velocity'] = {char(0x67,0x02),4},
	['current'] = {char(0x6D,0x02),2},
	['load'] = {char(0x6B,0x02),2},
	['voltage'] = {char(0x6F,0x02),2},
	['temperature'] = {char(0x71,0x02),1},

	-- External Data
	['data1'] = {char(0x72,0x02),2},
	['data2'] = {char(0x74,0x02),2},
	['data3'] = {char(0x76,0x02),2},
	['data4'] = {char(0x78,0x02),2},
	-- All data
	['data']  = {char(0x72,0x02),8},
	['indirect_data']  = {char(0x7A,0x02),1},

	-- Status return
	['status_return_level'] = {char(0x7B,0x03),1},
	['hardware_error'] = {char(0x7C,0x03),2},
}
libDynamixel.nx_registers = nx_registers

-- Assume MX and NX are at least similar
libDynamixel.registers_sensor = {
	'position', 'velocity', 'current', 'load', 'voltage', 'temperature',
}

function lD.parse(reg, status)
	if type(reg)~='string' or type(status)~='table' then return end
	status = status[1] and status[1] or status
	local reglocation = nx_registers[reg]
	if type(reglocation)~='table' then return end
	if type(status.parameter)~=table then return end
	return byte_to_number[reglocation[2]](unpack(status.parameter))
end
lD.parse_nx = lD.parse
function lD.parse_mx(reg, status)
	if type(reg)~='string' or type(status)~='table' then return end
	status = status[1] and status[1] or status
	local reglocation = mx_registers[reg]
	if type(reglocation)~='table' then return end
	if type(status.parameter)~=table then return end
	return byte_to_number[reglocation[2]](unpack(status.parameter))
end

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
		assert(nid==#data, 'Incongruent ids and data. nids:'..nid..' #:'..#data)
	end

	local t = {}
	local n = 1
	local len = 2 -- word
	for i = 1,nid do
		t[n] = ids[i];
		local val = all_data or data[i]
		-- Word to byte is the same for both packet types...
		t[n+1], t[n+2] = DP2.word_to_byte(val)
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
		assert(nid==#data, 'Incongruent ids and data. nids:'..nid..' #:'..#data)
	end
	local t = {}
	local n = 1
	for i = 1,nid do
		t[n] = ids[i]
		local val = all_data or data[i]
		t[n+1], t[n+2], t[n+3], t[n+4] = DP2.dword_to_byte(val)
		n = n + len + 1
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
byte_to_number[1] = function(byte) return byte end
byte_to_number[2] = DP2.byte_to_word
byte_to_number[4] = DP2.byte_to_dword
libDynamixel.byte_to_number = byte_to_number

-- Old get status method
local function get_status(fd, npkt, protocol, timeout)
	-- TODO: Is this the best default timeout for the new PRO series?
	timeout = timeout or READ_TIMEOUT
	npkt = npkt or 1
	local DP = protocol==1 and DP1 or DP2
	-- Form the temporary variables
	local status_str = ''
	--local full_status_str = ''
	local statuses = {}
	local in_timeout = true
	local tm0 = unix.time()
	--print('starting', t0)
	local s, pkts
	local max_i = 10
	local ii = 0
	while (in_timeout or s) and ii<=max_i do
		ii=ii+1
		--print('is_timeout', in_timeout, s, timeout)
		-- Wait for any packets
		--local status, ready = unix.select({fd},timeout)
		unix.select({fd},timeout)
		local tf = unix.time()
		--print("t_diff",tf-tm0,1/(tf-tm0))
		s = unix.read(fd)
		--print('s-type', type(s)=='string' and #s)
		if s then
			--full_status_str = full_status_str..s
			--print('Full Status str',#s,#full_status_str,pkts)
			pkts, status_str = DP.input(status_str..s)
			--print(type(pkts),'packets of',npkt)
			--print('PACKET', s:byte(1,-1))
			--print(#status_str, 'LEFTOVER:', status_str:byte(1,-1))
			--if type(pkts)=='table' then util.ptable(pkts) end
			if pkts then
				for p,pkt in ipairs(pkts) do
					table.insert(statuses, DP.parse_status_packet(pkt))
				end
				if #statuses>=npkt then return statuses end
			end -- if pkts
		end -- if s
		in_timeout = (unix.time()-tm0)<timeout
	end
	if ii >=10 then print('BAD ii!!') end
	--print('READ TIMEOUT',#statuses,'of',npkt,type(s),#status_str, ii)
	return statuses --, full_status_str
end

function libDynamixel.get_bulk(motor_ids, items, bus, is_co)
	local instruction = DP2.bulk_read(motor_ids, items)
	if not bus then return instruction end
	-- Clear the reading
	local buf
	repeat buf = unix.read(bus.fd) until not buf
	-- Write the instruction to the bus
	stty.flush(bus.fd)
	unix.write(bus.fd, instruction)
	if is_co then return end
	return get_status(bus.fd, #motor_ids)
end

function libDynamixel.set_bulk(motor_ids, items, vals, bus, is_co)
	local instruction = DP2.bulk_write(motor_ids, items, vals)
	if not bus then return instruction end
	-- Clear the reading
	unix.read(bus.fd)
	-- Write the instruction to the bus
	stty.flush(bus.fd)
	unix.write(bus.fd, instruction)
end

-- Set functions: returns the command to send on the chain
for k,v in pairs(nx_registers) do
	local addr, sz = unpack(v)
	local single_wr, sync_wr = nx_single_write[sz], sync_write[sz]
	libDynamixel['set_nx_'..k] = function(motor_ids, values, bus)
		-- Construct the instruction (single or sync)
		local single, instruction = type(motor_ids)=='number'
		if single then
			instruction = single_wr(motor_ids, addr, values)
			--[[
			elseif #motor_ids==1 then
			-- Single motor
			instruction =
			single_wr(motor_ids[1], addr, type(values)=='table' and values[1] or values)
			--]]
		else
			instruction =
			DP2.sync_write(addr, sz, char(unpack(sync_wr(motor_ids, addr, values))))
		end
		if not bus then return instruction end
		-- Clear the reading
		local buf
		repeat buf = unix.read(bus.fd) until not buf
		-- Write the instruction to the bus
		stty.flush(bus.fd)
		unix.write(bus.fd, instruction)
		-- Grab any status returns
		if using_status_return and single then
			return get_status(bus.fd, 1)
		end
	end --function
end

-- Indirect data
function libDynamixel.set_indirect_address(motor_ids, values, bus)
	assert(type(bus)=='table', 'need a bus for this one')
	local ptable = require'util'.ptable
	local addresses = {}
	for i, reg in ipairs(values) do
		local register = assert(nx_registers[reg], 'BAD INDIRECT REGISTER')
		local addr, sz = unpack(register)
		local base_addr = addr:byte(1) + 256 * addr:byte(2)
		for offset=0, sz-1 do
			local addr_num = base_addr + offset
			local high = addr_num % 256
			local low = (addr_num - high) / 256
			table.insert(addresses, {high, low})
		end
	end
	for i, id in ipairs(motor_ids) do
		local ind_base_addr, ind_offset = 49, 0
		for a, addr in ipairs(addresses) do
			local ind_addr = ind_base_addr + ind_offset
			local ind_high = ind_addr % 256
			local ind_low = (ind_addr - ind_high) / 256
			local addr_high, addr_low = unpack(addr)
			local addr_val = addr_high + addr_low * 256
			ind_offset = ind_offset + 2
			local instruction = DP2.write_word(
			id,
			char(ind_high, ind_low),
			addr_val
			)
			local buf
			repeat buf = unix.read(bus.fd) until not buf
			-- Write the instruction to the bus
			stty.flush(bus.fd)
			unix.write(bus.fd, instruction)
			local statuses = get_status(bus.fd, 1)
			for p, stat in pairs(statuses) do
				if stat.error>0 then
					print('BAD SET', id)
					ptable(stat)
				end
			end
		end
	end
end
function libDynamixel.check_indirect_address(motor_ids, values, bus)
	assert(type(bus)=='table', 'need a bus for this one')
	local ptable = require'util'.ptable
	local ok = true
	local addresses = {}
	for i, reg in ipairs(values) do
		local register = assert(nx_registers[reg], 'BAD INDIRECT REGISTER')
		local addr, sz = unpack(register)
		local base_addr = addr:byte(1) + 256 * addr:byte(2)
		for offset=0, sz-1 do
			local addr_num = base_addr + offset
			local high = addr_num % 256
			local low = (addr_num - high) / 256
			table.insert(addresses, {high, low})
		end
	end
	for i, id in ipairs(motor_ids) do
		local ind_base_addr, ind_offset = 49, 0
		for a, addr in ipairs(addresses) do
			local ind_addr = ind_base_addr + ind_offset
			local ind_high = ind_addr % 256
			local ind_low = (ind_addr - ind_high) / 256
			local addr_high, addr_low = unpack(addr)
			local addr_val = addr_high + addr_low * 256
			ind_offset = ind_offset + 2
			local instruction = DP2.read_data(id, char(ind_high, ind_low), 2)
			local buf
			repeat buf = unix.read(bus.fd) until not buf
			stty.flush(bus.fd)
			unix.write(bus.fd, instruction)
			local statuses = get_status(bus.fd, 1)
			for p, stat in pairs(statuses) do
				if stat.error~=0 or addr_high~=stat.parameter[1] or addr_low~=stat.parameter[2] then
					print('NO GOOD! MUST UPDATE', id)
					print('Current', unpack(stat.parameter))
					print('Desired', unpack(addr))
					ptable(stat)
					ok = false
				end
			end
		end
	end
	return ok
end

function libDynamixel.get_indirect_data(motor_ids, registers, bus, is_co)
	local ptable = require'util'.ptable
	local total_sz = 0
	local ind_data_addr_base = char(0x7A, 2)
	for i, reg in ipairs(registers) do
		local register = assert(nx_registers[reg], 'BAD INDIRECT REGISTER')
		local addr, sz = unpack(register)
		total_sz = total_sz + sz
	end

	-- Construct the instruction (single or sync)
	local instruction, nids
	if type(motor_ids)=='number' then
		-- Single motor
		instruction = DP2.read_data(motor_ids, ind_data_addr_base, total_sz)
		nids = 1
	else
		instruction =
		DP2.sync_read(char(unpack(motor_ids)), ind_data_addr_base, total_sz)
		nids = #motor_ids
	end

	-- If no pre-existing bus is specified, just return the instruction
	if not bus then return instruction end
	-- Clear old status packets
	local buf
	repeat buf = unix.read(bus.fd) until not buf
	-- Write the instruction to the bus
	stty.flush(bus.fd)
	unix.write(bus.fd, instruction)
	-- Grab the status of the register
	if is_co then
		return
	else	
		return get_status(bus.fd, nids)
	end

end

-- Get NX functions
-- Could be a coroutine
for k,v in pairs(nx_registers) do
	local addr, sz = unpack(v)
	libDynamixel['get_nx_'..k] = function(motor_ids, bus, is_co)
		-- Construct the instruction (single or sync)
		local instruction, nids
		if type(motor_ids)=='number' then
			-- Single motor
			instruction = DP2.read_data(motor_ids, addr, sz)
			nids = 1
		elseif type(motor_ids)=='table' and #motor_ids==1 then
			-- Single motor
			instruction = DP2.read_data(motor_ids[1], addr, sz)
		elseif type(motor_ids)=='table' then
			instruction = DP2.sync_read(char(unpack(motor_ids)), addr, sz)
			nids = #motor_ids
		else
			print('bad get nx')
			return
		end
		-- If no pre-existing bus is specified, just return the instruction
		if not bus then return instruction end
		-- Clear old status packets
		local buf
		repeat buf = unix.read(bus.fd) until not buf
		-- Write the instruction to the bus
		stty.flush(bus.fd)
		unix.write(bus.fd, instruction)
		-- Grab the status of the register
		if is_co then
			return
		else	
			return get_status(bus.fd, nids)
		end
	end --function
end

--------------------
-- Set MX functions
for k,v in pairs(mx_registers) do
	local addr, sz = unpack(v)
	libDynamixel['set_mx_'..k] = function(motor_ids, values, bus)
		-- Construct the instruction (single or sync)
		local single = type(motor_ids)=='number'
		local instruction = nil
		if single then
			instruction = mx_single_write[sz](motor_ids, addr, values)
		else
			instruction = DP2.sync_write(addr, sz, char(unpack(
			sync_write[sz](motor_ids, addr, values)
			)))
		end

		if not bus then return instruction end

		-- Write the instruction to the bus
		stty.flush(bus.fd)
		unix.write(bus.fd, instruction)

		-- Grab any status returns
		if using_status_return_mx and single then
			return get_status(bus.fd, 1)
			--[[
			local status = get_status(bus.fd)
			if not status then return end
			if status.error~=0 then return status end
			local value = byte_to_number[sz](unpack(status.parameter))
			return status, value
			--]]
		end

	end --function
end

--------------------
-- Get MX functions
for k,v in pairs(mx_registers) do
	libDynamixel['get_mx_'..k] = function(motor_ids, bus, is_co)
		local addr = v[1]
		local sz = v[2]

		-- Construct the instruction (single or sync)
		local instruction
		local nids = 1
		if type(motor_ids)=='number' then
			-- Single motor
			instruction = DP2.read_data(motor_ids, addr, sz)
		elseif #motor_ids==1 then
			-- Single motor
			instruction = DP2.read_data(motor_ids[1], addr, sz)
		else
			instruction = DP2.sync_read(char(unpack(motor_ids)), addr, sz)
			nids = #motor_ids
		end

		if not bus then return instruction end

		-- Clear old status packets
		unix.read(bus.fd)

		-- Write the instruction to the bus
		stty.flush(bus.fd)
		unix.write(bus.fd, instruction)

		-- Grab the status of the register
		if is_co then
			return
		else
			return get_status(bus.fd, nids)
		end

	end --function
end

--------------------
-- Set RX functions
for k,v in pairs(rx_registers) do
	libDynamixel['set_rx_'..k] = function(motor_ids, values, bus)
		local addr = v[1]
		local sz = v[2]

		-- Construct the instruction (single or sync)
		local single = type(motor_ids)=='number'
		local instruction = nil
		if single then
			instruction = rx_single_write[sz](motor_ids, addr, values)
		else
			local msg = sync_write[sz](motor_ids, addr, values)
			instruction = DP1.sync_write(addr, sz, char(unpack(msg)))
		end

		if not bus then return instruction end

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
-- Get RX functions
for k,v in pairs(rx_registers) do
	libDynamixel['get_rx_'..k] = function( motor_id, bus )
		local addr = v[1]
		local sz = v[2]
		-- Single motor at a time for reading
		local instruction = DP1.read_data(motor_id, addr, sz)
		if not bus then return instruction end
		-- Clear old status packets
		local clear = unix.read(bus.fd)
		-- Write the instruction to the bus
		stty.flush(bus.fd)
		local ret = unix.write(bus.fd, instruction)
		-- Grab the status of the register
		local status = get_status( bus.fd, 1, 1 )
		-- If not data from the chain
		if not status then return end
		local value
		if sz==8 and #status.parameter==8 then
			-- Everything!
			value = {}
			-- In steps
			value.position = byte_to_number[2]( unpack(status.parameter,1,2) )
			local speed = byte_to_number[2]( unpack(status.parameter,3,4) )
			if speed>=1024 then speed = 1024-speed end
			-- To Radians per second
			value.speed = (speed * math.pi) / 270
			local load  = byte_to_number[2]( unpack(status.parameter,5,6) )
			if load>=1024 then load = 1024-load end
			-- To percent
			value.load = load / 10.24
			-- To Volts
			value.voltage = status.parameter[7] / 10
			-- Is Celsius already
			value.temperature = status.parameter[8]
		else
			local ff = byte_to_number[#status.parameter]
			if not ff then return end
			value = ff( unpack(status.parameter) )
		end
		return status, value
	end --function
end

--------------------
-- Ping functions
local function send_ping( bus, id, protocol, twait )
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
	if type(status)=='table' then return status end
end


local ptable = require'util'.ptable
local function check_motor_ping(status_tbl)
	--print(status_tbl)
	if type(status_tbl)~='table' then return end
	--print('check', #status_tbl)
	--ptable(status_tbl)
	local status = unpack(status_tbl) or status_tbl
	if type(status)~='table' then return end
	local params = status.parameter
	if type(params)~='table' then return end
	local lsb, msb = unpack(params)
	--print('Found', status.id, status.error>0 and status.error or 'ok')
	if msb>2 then return 'nx' else return 'mx' end
end

local function debug_mx(self, id)
	local parse_firmware = byte_to_number[mx_registers.firmware[2]]
	local parse_delay = byte_to_number[mx_registers.return_delay_time[2]]
	local parse_mode = byte_to_number[mx_registers.mode[2]]
	local parse_status_return_level = byte_to_number[mx_registers.status_return_level[2]]
	-- Check the firmware
	local status = lD.get_mx_firmware(id, self)
	if type(status)=='table' and type(status[1])=='table' then
		local firmware = parse_firmware(unpack(status[1].parameter))
		print('\tFirmware: '..firmware)
	end
	-- Check the Operating Mode
	status = lD.get_mx_mode(id, self)
	if type(status)=='table' and type(status[1])=='table' then
		local mode = parse_delay(unpack(status[1].parameter))
		print('\tOperating Mode: '..mode)
	end
	-- Return Delay
	status = lD.get_mx_return_delay_time(id, self)
	if type(status)=='table' and status[1] then
		status = status[1]
		local delay = parse_delay(unpack(status.parameter))
		print('\tReturn Delay: '..delay)
	end
	status = lD.get_mx_status_return_level(id, self)
	if type(status)=='table' and status[1] then
		status = status[1]
		local level = parse_status_return_level(unpack(status.parameter))
		print('\tStatus Return Level: '..level)
	end


end

local function debug_nx(self, id)
	local parse_firmware = byte_to_number[nx_registers.firmware[2]]
	local parse_delay = byte_to_number[nx_registers.return_delay_time[2]]
	local parse_mode = byte_to_number[nx_registers.mode[2]]
	local parse_status_return_level = byte_to_number[nx_registers.status_return_level[2]]
	-- Check the firmware
	local status = lD.get_nx_firmware(id, self)
	if type(status)=='table' and type(status[1])=='table' then
		local mode = parse_firmware(unpack(status[1].parameter))
		print('\tFirmware: '..mode)
	end
	-- Check the Operating Mode
	status = lD.get_nx_mode(id, self)
	if type(status)=='table' and type(status[1])=='table' then
		local mode = parse_mode(unpack(status[1].parameter))
		print('\tOperating Mode: '..mode)
	end
	-- Return Delay
	status = lD.get_nx_return_delay_time(id, self)
	if type(status)=='table' and status[1] then
		status = status[1]
		local delay = parse_delay(unpack(status.parameter))
		print('\tReturn Delay: '..delay)
	end
	status = lD.get_nx_status_return_level(id, self)
	if type(status)=='table' and status[1] then
		status = status[1]
		local level = parse_status_return_level(unpack(status.parameter))
		print('\tStatus Return Level: '..level)
	end
end

local function ping_probe(self, protocol, twait)
	local found_ids = {}
	local mx_ids, has_mx_id = {}, {}
	local nx_ids, has_nx_id = {}, {}
	protocol = protocol or 2
	twait = twait or READ_TIMEOUT
	for id=1, 253 do
		local status = send_ping(self, id, protocol, twait)
		local ok = check_motor_ping(status)
		if ok then
			print('Found', ok, id)
			table.insert(found_ids, id)
			if ok=='nx' then
				table.insert(nx_ids, id)
				has_nx_id[id] = true
				debug_nx(self, id)
			else
				table.insert(mx_ids, id)
				has_mx_id[id] = true
				debug_mx(self, id)
			end
		end
		-- Wait .1 ms
		unix.usleep(READ_TIMEOUT * 1e6)
	end
	self.m_ids = found_ids
	self.mx_ids = mx_ids
	self.nx_ids = nx_ids
	self.has_mx_id = has_mx_id
	self.has_nx_id = has_nx_id
	self.has_mx = #mx_ids > 0
	self.has_nx = #nx_ids > 0
	return found_ids
end

local function ping_verify(self, m_ids, protocol, twait)
	local allgood = true
	local found_ids = {}
	local mx_ids, has_mx_id = {}, {}
	local nx_ids, has_nx_id = {}, {}
	local nx_cmd_ids, mx_cmd_ids = {}, {}
	local has_nx_cmd_id, has_mx_cmd_id = {}, {}
	protocol = protocol or 2
	twait = twait or READ_TIMEOUT
	for i, id in ipairs(m_ids) do
		local status = send_ping(self, id, protocol, twait)
		local ok = check_motor_ping(status)
		if ok then 
			table.insert(found_ids, id)
			if ok=='nx' then
				table.insert(nx_ids, id)
				has_nx_id[id] = true
				--debug_nx(self, id)
				table.insert(nx_cmd_ids, id)
				has_nx_cmd_id[id] = true
			else
				table.insert(mx_ids, id)
				has_mx_id[id] = true
				--debug_mx(self, id)
				table.insert(mx_cmd_ids, id)
				has_mx_cmd_id[id] = true
			end
		else
			print('NOT FOUND:', id, ok)
			if id<32 then
				--nx
				table.insert(nx_cmd_ids, id)
				has_nx_cmd_id[id] = true
			else
				-- mx
				table.insert(mx_cmd_ids, id)
				has_mx_cmd_id[id] = true
			end
			allgood = false
		end
		-- Wait .1 ms
		unix.usleep(READ_TIMEOUT * 1e6)
	end
	self.m_ids = found_ids
	self.mx_ids = mx_ids
	self.nx_ids = nx_ids
	self.has_mx_id = has_mx_id
	self.has_nx_id = has_nx_id
	self.has_mx = #mx_ids > 0
	self.has_nx = #nx_ids > 0
	--
	self.mx_cmd_ids = mx_cmd_ids
	self.nx_cmd_ids = nx_cmd_ids
	self.has_mx_cmd_id = has_mx_cmd_id
	self.has_nx_cmd_id = has_nx_cmd_id
--	lD.set_nx_status_return_level(nx_cmd_ids, 1, self)

	if allgood then print('VERIFID') end
	return allgood
end

local function close(self)
	return unix.close(self.fd)==0
end

local function reset(self)
	self:close()
	unix.usleep( 1e3 )
	self.fd = libDynamixel.open(self.ttyname)
end

local function send_instruction(bus, instruction)
	local fd, buf = bus.fd
	-- Clear the reading
	repeat buf = unix.read(fd) until not buf
	-- Write the instruction to the bus
	stty.flush(fd)
	return unix.write(fd, instruction)
end

--------------------
-- Generator of a new bus
function libDynamixel.new_bus(ttyname, ttybaud)
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
	local obj = {
		fd = fd,
		ttyname = ttyname,
		baud = baud,
		close = close,
		reset = reset,
		ping_probe = ping_probe,
		ping_verify = ping_verify,
		ping = send_ping,
		send_instruction = send_instruction,
	}

	return obj
end

function libDynamixel.tostring(instruction)
	local instruction_bytes = {}
	local instruction_ints = {}
	for i, v in ipairs({instruction:byte(1,-1)}) do
		table.insert(instruction_bytes, string.format(' %02X', v))
		table.insert(instruction_ints, string.format('%3d', v))
	end
	return table.concat(instruction_bytes, ' '), table.concat(instruction_ints, ' ')
end

return libDynamixel
