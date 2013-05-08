local libDynamixel = require('libDynamixel');

local use_real_device = true
local test_probe = false
local test_torque = false
local test_position = false
local show_pairs = false
local test_crc = false
local test_led = false
local test_read_led = true
local test_sync_led = false

-- Fake device for testing?
local dev_name = 'fake'
if use_real_device then
	dev_name = nil
end
local Dynamixel = libDynamixel.open( dev_name )

if test_crc then
	local DynamixelPacket = require('DynamixelPacket');
	local pkt = string.char(255,255,253,0,7,6,0,3,24,0,0)
	h,l=DynamixelPacket.crc16( pkt )
	print(string.format('crc: %x %x %d',l,h, #pkt) )
end

if show_pairs then
	for k,v in pairs(libDynamixel) do
		print(k,v)
	end
	for k,v in pairs(Dynamixel) do
		print(k,v)
	end
end

if test_probe then
--	local twait = 0.010
	local twait = 0.020
	Dynamixel:ping_probe(twait);
end

if test_led then
	for j=14,18,2 do
		print('LED ',j)
		Dynamixel:set_mx_led( j, 1 )
		os.execute('sleep 0.01')
	end
end

if test_read_led then
	local id = 16
	local res = Dynamixel:get_mx_led( id )
	print('Reading LED',id,res)
end

if test_sync_led then
	local ids = {14,16,18}
	local val = 0
	if val==0 then
		print('Sync LEDs OFF',unpack(ids))
	else
		print('Sync LEDs ON',unpack(ids))
	end
	Dynamixel:set_mx_led( ids, val )
end

if test_torque then
	print('Testing torque enable with MX motors')
	local val = 0
	for id=14,14 do
		io.write(string.format('ID %d\n', id) )
		local ret = Dynamixel:set_mx_torque_enable(id,val)
		io.write( string.format('Sent %d bytes.\n',ret) )
		print()
	end
end

if test_position and false then
	local goal = 72000 --4096
	print('Testing position with MX motors. Goal:',goal)
	for id=7,9 do
		io.write(string.format('ID %d\n', id) )
		local ret = Dynamixel:set_nx_command_position(id,goal)
		io.write( string.format('Sent %d bytes.\n',ret) )
		print()
	end
end

-- Close at the end of the script
Dynamixel:close()