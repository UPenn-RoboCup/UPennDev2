local ffi = require'ffi'
local C = ffi.C
local bit = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local band   = bit.band
local bor    = bit.bor
local char = string.char

local DP2 = {}

ffi.cdef[[
	static const int MAXNUM_TXPARAM = 65535;
	static const int MAXNUM_RXPARAM = 65535;
]]

ffi.cdef[[
	static const int N_PACKET_HEADERS = 6;
	static const uint8_t DYNAMIXEL_PACKET_HEADER  = 255;
	static const uint8_t DYNAMIXEL_PACKET_HEADER_2  = 255;
	static const uint8_t DYNAMIXEL_PACKET_HEADER_3  = 253;
	static const uint8_t DYNAMIXEL_PACKET_STUFFING  = 0;
]]

local N_PACKET_HEADERS = 6;
local DYNAMIXEL_PACKET_HEADER  = 255;
local DYNAMIXEL_PACKET_HEADER_2  = 255;
local DYNAMIXEL_PACKET_HEADER_3  = 253;
local DYNAMIXEL_PACKET_STUFFING  = 0;

ffi.cdef[[
	/* Packet Struct */
	typedef struct DynamixelPacket {
		uint8_t header1;
		uint8_t header2;
		uint8_t header3;
		uint8_t stuffing;
		uint8_t id;
		uint8_t len[2]; /* length does not include first 7 bytes */
		/* DONE HEADER */
		uint8_t instruction; // or error for status packets
		uint8_t parameter[MAXNUM_TXPARAM]; // reserve for maximum packet size

		uint16_t checksum; // Needs to be copied at end of parameters
		uint16_t length; // Needs to be copied at end of parameters
	} DynamixelPacket;
]]

local function pkt2tbl(raw, obj)
	local pkt = ffi.string(raw, obj.length+N_PACKET_HEADERS)
	local err, len = pkt:byte(9), pkt:byte(6) + 256 * pkt:byte(7)
	if err>0 then
		return {
			id = pkt:byte(5),
			length = len,
			instruction = pkt:byte(8),
			error = err,
			checksum = char(pkt:byte(10), pkt:byte(11))
		}
	else
		return {
			id = pkt:byte(5),
			length = len,
			instruction = pkt:byte(8),
			error = err,
			parameter = {pkt:byte(10, len+5)},
			raw_parameter = pkt:sub(10, len+5),
			checksum = char(pkt:byte(len+6), pkt:byte(len+7))
		}
	end
end

-- This is a coroutine for processing any inputs from the bus
-- Input: bus is the bus Table
-- Input: buf is String of data from the bus
-- Input: npkt is the Number of packets we expect
function DP2.input_co(bus, buf)
	local rx_i, npkt_processed = 0, 0
	if not bus or type(buf)~='string' or bus.npkt_to_expect < 1 then
		return
	end
	local pkt_obj, pkt_raw = bus.rx_pkt, bus.rx_raw
	local raw, n = ffi.cast('uint8_t*', buf), #buf
	coroutine.yield()
::LBL_PROC_INPUT::
	for i=1, n do
		-- Populate the packet
		local c = raw[0]
		raw = raw + 1
		pkt_raw[rx_i] = c
		if rx_i < N_PACKET_HEADERS then
			if rx_i == 0 and c ~= DYNAMIXEL_PACKET_HEADER then
				-- Start a new packet
				rx_i = -1
			elseif rx_i == 1 and c ~= DYNAMIXEL_PACKET_HEADER_2 then
				-- Start a new packet
				rx_i = -1
			elseif rx_i == 2 and c ~= DYNAMIXEL_PACKET_HEADER_3 then
				-- Start a new packet
				rx_i = -1
			else
				-- Ignore
			end
		elseif rx_i == N_PACKET_HEADERS then
			-- Set the length of the packet
			pkt_obj.length = pkt_obj.len[0] + band(lshift(pkt_obj.len[1], 8), 0xFF00)
		elseif rx_i == pkt_obj.length + N_PACKET_HEADERS then
			-- TODO: Verify the checksum
			-- if good checksum, then we have a packet!
			local tbl = pkt2tbl(pkt_raw, pkt_obj)
			npkt_processed = npkt_processed + 1
			if npkt_processed==bus.npkt_to_expect then
				return tbl
			else
				coroutine.yield(tbl)
			end
			-- Start a new packet
			rx_i = -1
		elseif rx_i > pkt_obj.length + N_PACKET_HEADERS then
			-- Start a new packet
			rx_i = -1
			-- TODO: I think this misses some stuff...
			error('DP2 | PACKET OVERFLOW')
		end
		-- Increment the index
		rx_i = rx_i + 1
	end
	-- Check the number of packets we processed
	if npkt_processed >= bus.npkt_to_expect then return end
	-- Check if we are time'd out
	local t = unix.time()
	if t > bus.read_to then return end
	-- Within time limit, and still not received enough packets
	-- Everything is saved in the packet
	bus, buf = coroutine.yield()
	assert(buf, 'NO BUF ON WITHIN_TIMEOUT')
	raw, n = ffi.cast('uint8_t*', buf), #buf
	goto LBL_PROC_INPUT
end

function DP2.init_bus(bus)
	bus.rx_pkt = ffi.new'DynamixelPacket'
	bus.rx_raw = ffi.cast('uint8_t*', bus.rx_pkt)
end

return DP2
