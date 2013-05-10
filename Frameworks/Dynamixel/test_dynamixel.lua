local libDynamixel = require('libDynamixel');

local use_real_device = true
local test_probe = false
local show_pairs = false
local test_crc = false
local test_led = false
local test_sync_led = false
local test_read_position = false
local test_sync_read_position = true
local test_torque = false
local test_command_position = true
local test_error = false
local arm_side = 'left' -- Even numbers
--local arm_side = 'right' -- Odd numbers

-- Populate the IDs
local right_nx_ids = {}
local right_mx_ids = {}
local right_all_ids = {}
local left_nx_ids = {}
local left_mx_ids = {}
local left_all_ids = {}

for i=1,18,2 do
	if i<12 then
		table.insert(right_nx_ids,i)
		table.insert(left_nx_ids,i+1)
	else
		table.insert(right_mx_ids,i)
		table.insert(left_mx_ids,i+1)
	end
	table.insert(right_all_ids,i)
	table.insert(left_all_ids,i+1)
end

-- Fake device for testing?
local dev_name = 'fake'
if use_real_device then
	dev_name = nil
end
local Dynamixel = libDynamixel.new_bus( dev_name )
os.execute('sleep 0.01')

if test_crc then
	local DynamixelPacket = require('DynamixelPacket');
	local pkt = string.char(255,255,253,0,7,6,0,3,24,0,0)
	h,l=DynamixelPacket.crc16( pkt )
	print(string.format('crc: %x %x %d',l,h, #pkt) )
end

if test_torque then
	local val = 1
	print('\nTesting torque enable with NX motors',val)
	local my_ids = left_nx_ids
	for idx,id in ipairs(my_ids) do
		
		local ret = Dynamixel:set_nx_torque_enable(id,val)
		io.write(string.format('ID %d sent %d bytes.\n', id,ret) )
		os.execute('sleep 0.02')
		print()
	end
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
	local ids = left_nx_ids
	local val = 255
	if val==0 then
		print('Sync LEDs OFF',unpack(ids))
	else
		print('Sync LEDs ON',unpack(ids))
	end
	Dynamixel:set_nx_led_red( ids, val )
end

if test_command_position and true then
	local goal = 0
	local ids = {7,9,11}
	print('Testing position with NX motors. Goal:',goal)
	os.execute('sleep 0.01')
	local ret = Dynamixel:set_nx_torque_enable(ids,1)
	os.execute('sleep 0.01')
	local ret = Dynamixel:set_nx_command_position(ids,goal)
	io.write( string.format('Sent %d bytes.\n',ret) )
end

if test_sync_read_position then
	local my_ids = right_nx_ids
	os.execute('sleep 0.01')
	print('\nSync Reading Position of IDs', unpack(my_ids) )
	local res = Dynamixel:get_nx_torque_enable( my_ids )
	if res then
		for idx,res2 in ipairs(res) do
			io.write( string.format('ID %2d: %d\n',my_ids[idx], res2 ) )
		end
	end
end

if test_command_velocity and true then
	local id = 8
	local ret = Dynamixel:set_nx_mode(id,0)--torque
	local goal = 100;
	
	local ret = Dynamixel:set_nx_command_torque(id,goal)
	os.execute('sleep 0.01')
	io.write( string.format('Sent %d bytes.\n',ret) )
end

if test_error then
	local err_code = string.char(1)
	local DynamixelPacket = require('DynamixelPacket');
	local err_msg = DynamixelPacket.strerr(err_code)
	for i,k in ipairs(err_msg) do
		print('Error test:',k)
	end
end

-- Close at the end of the script
Dynamixel:close()