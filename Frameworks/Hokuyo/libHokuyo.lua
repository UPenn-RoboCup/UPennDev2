-- libHokuyo
-- (c) 2013 Stephen McGill, Yida Zhang
-- Hokuyo Library

local libHokuyo = {}
local HokuyoPacket = require 'HokuyoPacket'
local cutil = require 'cutil'
local stty = require 'stty'
local carray = require 'carray'

--------------------
-- Paramteres for the different dievices
local HOKUYO_SCAN_REGULAR = 0
local HOKUYO_2DIGITS = 2
local HOKUYO_3DIGITS = 3

local write_check = function(fd, cmd, resp_tbl)
		
	--------------------
	-- Write the command
	local ret = unix.write(fd, cmd)
	-- Gather the response from the hokuyo
	local res = ''
	local iCmd = nil
	local t_write = unix.time()
	while true do
		local res1 = unix.read(fd)
		if type(res1)=='string' then
			res = res..res1
			iCmd = res:find(cmd)
			if iCmd then
				unix.usleep(1000)
				local append = unix.read(fd)
				if append then
					res = res..append
				end
				break
			else
				res = ''
			end
		else
			local t_diff = unix.time() - t_write
			if t_diff>2 then
				error('Timeout opening the device!')
			end
		end
		
	end
	assert(iCmd,string.format('Echo of command (%s) not received',cmd))
	--------------------

	--------------------
	-- Check the correctness of the response
	-- Filter to only the response
	local response = res:sub(iCmd+#cmd)
	local end_of_reponse = response:find('\n')
	assert(end_of_reponse,'Cannot find the end of the response')
	--------------------
  
	--------------------
	-- Assume a good response until otherwise checked
	for i = 1, #resp_tbl do
		local expected = resp_tbl[i]
		local actual = response:sub(1, #expected)
		assert(
		actual==expected,
		string.format('Bad response. Expected (%s) Got (%s) for Argument %d', 
		expected,actual,i )
		)
	end
	--------------------

	return response
end

local create_scan_request = 
function(scan_start, scan_end, scan_skip, encoding, scan_type, num_scans)
-- TODO range checks and more types support
if num_scans==0 then
	if scan_type == HOKUYO_SCAN_REGULAR then
		if encoding == HOKUYO_3DIGITS then
			req = string.format(
			"MD%04d%04d%02x0%02d\n", scan_start,scan_end,scan_skip,num_scans
			);
			return req
			elseif (encoding == HOKUYO_2DIGITS) then
				req = string.format(
				"MS%04d%04d%02x0%02d\n", scan_start,scan_end,scan_skip,num_scans
				);
				return req
			else
				error('invalid selection of character encoding')
			end
		end
	end

end

local stream_off = function(self)
	local ntries = 5
	local stop_delay = 5e4
	for i=1,ntries do
		local resp = write_check(self.fd, 'QT\n', {'00'})
		if resp then
			break
		end
		unix.usleep(stop_delay)
	end
end

local stream_on = function(self)
	
	-- Ensure that we stream_off before we stream_on again
	stream_off(self)

	-- Response should be one of these: '00', '02'
	local ret = write_check(self.fd, 'BM\n', {'00'})

	-- scan_start, scan_end, scan_skip, encoding, scan_type, num_scans 
	local scan_req = create_scan_request(0, 1080, 1, 3, 0, 0);
	if scan_req then
		write_check(self.fd, scan_req, {'00'})
	end

end

local get_sensor_params = function(self)
	local res = write_check(self.fd, 'PP\n', {'00'})
	assert(res,'Could not get sensor parameters!')
	params = HokuyoPacket.parse_info(res, 8)
	local sensor_params = {}

	sensor_params.model = params[1]
	sensor_params.min_distance = tonumber(params[2])
	sensor_params.max_distance = tonumber(params[3])
	sensor_params.angular_resolution = tonumber(params[4])
	sensor_params.min_angle_count = tonumber(params[5])
	sensor_params.max_angle_count = tonumber(params[6])
	sensor_params.frint_angle_count = tonumber(params[7])
	sensor_params.scan_rate = tonumber(params[8])

	return sensor_params
end

local get_sensor_info = function(self)
	local cmd = 'VV\n'
	local resp = '00'
	local res = write_check(self.fd, cmd, {resp})
	if not res then return end
	info =	HokuyoPacket.parse_info(res, 5)

	local sensor_info = {}
 
	sensor_info.vender = info[1]
	sensor_info.product = info[2]
	sensor_info.firmware = info[3]
	sensor_info.protocol = info[4]
	sensor_info.serial_number = info[5]
	return sensor_info
end

local set_baudrate = function(self, baud)
	local cmd = 'SS'..string.format("%6d", baud)..'\n';
	-- Should get one of these... '00', '03', '04'
	local res = write_check(self.fd, cmd, {'04'})
end

buf = ''
raw_scan = ''
-- Grab the scan from the buffer
local get_scan = function(self)
	--  local t0 = unix.time()
	while true do
		raw_scan = unix.read(self.fd, 3372-#buf )
		--raw_scan = unix.read(self.fd, 3372 )
		if raw_scan then
			raw_scan = buf..raw_scan
			if #raw_scan==3372 then
				break
			end
			-- We may need to try again!
			buf = raw_scan
		end
		-- FASTER data
		--unix.select({self.fd},0.050)
		-- LESS CPU, SLOWER DATA
		--unix.usleep(12100)
		-- BEST OF BOTH
		if not raw_scan or #raw_scan>2000 then
			unix.select({self.fd},0.050)
		else
			unix.usleep(11000)
		end

	end
	-- Reset the buffer
	buf = ''
	--  local t1 = unix.time()
	--  print( string.format('Took %5.2f ms to get data\n',1000*(t1-t0)))
	return HokuyoPacket.parse(raw_scan)
end

current_lidar = 0
max_num_lidar = 255
local function next_lidar()
	while	os.execute("test -c /dev/ttyACM"..current_lidar)~=0 do
		current_lidar = current_lidar + 1
		if current_lidar>max_num_lidar then
			error('Could not find a lidar!')
		end
	end
	local device = "/dev/ttyACM"..current_lidar
	current_lidar = current_lidar + 1
	return device
end

libHokuyo.open = function(ttyname, serial, ttybaud )
	local baud = ttybaud or 115200
	local name = ttyname or next_lidar()
	unix = require 'unix'

	-----------
	-- Open the Serial device with the proper settings
	print( name, 'Opening with baud', baud)
	local fd = unix.open( name, unix.O_RDWR + unix.O_NOCTTY)
	assert(fd > 2, string.format("Could not open port %s, (%d)\n", name, fd))
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)
	assert(fd > 2,
	string.format("Could not set port parameters %s, (%d)\n", name, fd)
	)
	-----------

	-----------
	-- Begin the Hokuyo object
	local obj = {}
	-----------
	
	-----------
	-- Set the serial port data
	-----------
	obj.fd = fd
	obj.ttyname = name
	obj.baud = baud
	obj.close = function(self)
		return unix.close(self.fd) == 0
	end

	-----------
	-- Get the sensor parameters
	-----------
	print(name, 'Get sensor parameters')
	obj.params = get_sensor_params(obj)
	if not obj.params then
		obj:close()
		error()
	end
	
	-----------
	-- Get the sensor information
	-----------
	print(name, 'Get sensor info')
	--unix.usleep(1E6 * 0.05)
	obj.info = get_sensor_info(obj)
	if not obj.info then
		obj:close()
		error()
	end

	-----------
	-- Set the baud rate
	print(name, 'Set baud rate')
	--unix.usleep(1E6 * 0.05)
	set_baudrate(obj, baud)
	-----------
	
	-----------
	-- Set the methods for accessing the data
	-----------
	obj.stream_on = stream_on
	obj.stream_off = stream_off
	obj.get_scan = get_scan
	-----------
	
	-----------
	-- Return the hokuyo object
	return obj
	-----------
end

return libHokuyo
