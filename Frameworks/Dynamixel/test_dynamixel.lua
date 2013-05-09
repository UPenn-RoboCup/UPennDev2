local libDynamixel = require('libDynamixel');

local use_real_device = true
local test_probe = false
local test_torque = false
local test_position = false
local show_pairs = false
local test_crc = false
local test_led = false
local test_sync_led = false
local test_read_position = true
local test_sync_read_position = true

-- Fake device for testing?
local dev_name = 'fake'
if use_real_device then
	dev_name = nil
end
local Dynamixel = libDynamixel.new_bus( dev_name )

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
	local twait = 0.020
	Dynamixel:ping_probe(twait);
end

if test_led then
	os.execute('sleep 0.01')
	local val = 255
	if val==0 then
		print('Sync LEDs OFF')
	else
		print('Sync LEDs ON')
	end
	for j=1,12 do
		print('LED ',j)
		Dynamixel:set_nx_led_red( j, val )
		os.execute('sleep 0.01')
	end
end

if test_read_position then
	local id = 15
	os.execute('sleep 0.01')
	print('\nReading Position of ID',id)
	local res = Dynamixel:get_mx_position( id )
	print('Result',res)
end

if test_sync_led then
	local ids = {13,14,15,16,17,18}
	local val = 0
	if val==0 then
		print('Sync LEDs OFF',unpack(ids))
	else
		print('Sync LEDs ON',unpack(ids))
	end
	Dynamixel:set_mx_led( ids, val )
end

if test_sync_read_position then
	local ids = {13,15,17}
--	local ids = {14,16,18}
	os.execute('sleep 0.01')
	print('\nSync Reading Position of IDs', unpack(ids) )
	local res = Dynamixel:get_mx_position( ids )
	print('Result',res)
end

if test_torque then
	print('Testing torque enable with MX motors')
	local val = 1
	for id=14,18,2 do
		io.write(string.format('ID %d\n', id) )
		local ret = Dynamixel:set_mx_torque_enable(id,val)
		io.write( string.format('Sent %d bytes.\n',ret) )
		os.execute('sleep 0.01')
		print()
	end
end

if test_position and false then
	local goal = 2048
	print('Testing position with MX motors. Goal:',goal)
	os.execute('sleep 0.01')
	for id=16,16 do
		io.write(string.format('Position ID %d\n', id,goal) )
		local ret = Dynamixel:set_mx_command(id,goal)
		io.write( string.format('Sent %d bytes.\n',ret) )
		print()
	end
end

-- Close at the end of the script
Dynamixel:close()