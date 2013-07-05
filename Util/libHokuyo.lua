-- libHokuyo
-- (c) 2013 Stephen McGill, Yida Zhang
-- Hokuyo Library

local libHokuyo = {}
local HokuyoPacket = require 'HokuyoPacket'
local stty = require 'stty'
local unix = require 'unix'

--------------------
-- libHokuyo constants and internal parameters
HOKUYO_SCAN_REGULAR = 0
HOKUYO_2DIGITS = 2
HOKUYO_3DIGITS = 3
libHokuyo.CURRENT_HOKUYO = -1
libHokuyo.MAX_NUM_HOKUYO = 255

local write_command = function(fd, cmd, expected_resp)
	-- Should get one of these... '00', '02', '03', '04'
	expected_resp = expected_resp or '00'

	--------------------
	-- Write the command
	local ret = unix.write(fd, cmd)
	-- TODO: What is the expected response time? Should yield this
	-- Gather the response from the hokuyo
	local response = ''
	local response_timeout = 2
	local iCmd = nil
	local t_write = unix.time()
	while not iCmd do
		-- TODO: Sleep for each loop in wait of the response
		-- Ensure we get a prompt response
		local t_diff = unix.time() - t_write
		assert( t_diff<response_timeout, 
		string.format('Response timeout for command (%s)',cmd)
		)
		-- Read from the Hokuyo
		response = unix.read(fd)
		if type(response)=='string' then
			iCmd = res:find(cmd)
		end
	end
	--------------------

	--------------------
	-- Check the correctness of the response
	-- Filter to only the response
	response = response:sub(iCmd+#cmd)
	local end_of_reponse = response:find('\n')
	-- Wait for and append more data if we cannot find the full response
	if not end_of_response then
		-- TODO: Yield an expected response time.  Have a smart sleep time
		unix.usleep(1000)
		local append = unix.read(fd)
		assert( type(append)=='string', 'Did not receive appended response' )
		response = response..append
	end
end_of_reponse = response:find('\n')
assert(end_of_reponse,'Cannot find the end of the response')
--------------------

--------------------
-- Assume a good response until otherwise checked
local actual = response:sub(1, 2)
assert(
actual==expected_response,
string.format('Expected response (%s) but got (%s)', 
expected_response, actual )
)
--------------------

return response
end

local create_scan_request = 
function(scan_start, scan_end, scan_skip, encoding, scan_type, num_scans)
	-- TODO range checks and more types support
	local request = nil
	local base_request = string.format(
	"MD%04d%04d%02x0%02d\n", scan_start,scan_end,scan_skip,num_scans
	);
	if num_scans==0 then
		if scan_type == HOKUYO_SCAN_REGULAR then
			if encoding == HOKUYO_3DIGITS then
				request = 'MD'..base_request
			elseif encoding == HOKUYO_2DIGITS then
				request = 'MS'..base_request
			end
		end
	end
	assert(request,'Invalid character encoding')
	return request
end

local stream_off = function(self)
	local ntries = 5
	local stop_delay = 5e4
	for i=1,ntries do
		local resp = write_command( self.fd, 'QT\n' )
		if resp then
			break
		end
		-- TODO: Yield the delay time between stop attempts
		unix.usleep(stop_delay)
	end
end

local stream_on = function(self)

	-- Ensure that we stream_off before we stream_on again
	stream_off(self)

	local ret = write_command( self.fd, 'BM\n' )

	-- scan_start, scan_end, scan_skip, encoding, scan_type, num_scans 
	-- TODO: Check encoding types
	local scan_req = create_scan_request(0, 1080, 1, 3, 0, 0);
	-- TODO: Does the scan request expect a return, 
	-- or just the actual streaming data?
	write_command( self.fd, scan_req )

end

local get_sensor_params = function(self)
	local res = write_command( self.fd, 'PP\n' )

	local sensor_params = {}
	local params = HokuyoPacket.parse_info(res, 8)
	--[[
	if not obj.params then
	obj:close()
	error('Could not set Hokuyo sensor parameters')
	end
	--]]
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
	local res = write_command( self.fd, cmd )

	local sensor_info = {}
	local info = HokuyoPacket.parse_info(res, 5)
	--[[
	if not obj.info then
	obj:close()
	error()
	end
	--]]
	sensor_info.vender = info[1]
	sensor_info.product = info[2]
	sensor_info.firmware = info[3]
	sensor_info.protocol = info[4]
	sensor_info.serial_number = info[5]
	return sensor_info
end

---------------------------
-- Set the Hokuyo baud rate
local set_baudrate = function(self, baud)
	local cmd = 'SS'..string.format("%6d", baud)..'\n';
	local res = write_command(self.fd, cmd, '04')
end

---------------------------
-- Grab the scan from the buffer
-- TODO: Add a timeout
local get_scan = function(self)
	--  local t0 = unix.time()
	local raw_scan = ''
	local buf = ''
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
		-- TODO: Yield expected response time
		-- FASTER data
		--unix.select({self.fd},0.050)
		-- LESS CPU, SLOWER DATA
		--unix.usleep(12100)
		-- BEST OF BOTH
		-- TODO: Debug how many times each is called
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

---------------------------
-- Acquire the next Hokuyo
libHokuyo.next_hokuyo = function(self, serial, ttybaud)
	local current_lidar = libHokuyo.CURRENT_HOKUYO
	-- TODO: Prefix: Allow for usage on a Mac, too
	local hokuyo_prefix = "/dev/ttyACM"
	local found_device = nil
	while not found_device and current_lidar<self.MAX_NUM_HOKUYO do
		current_lidar = current_lidar + 1
		local device = hokuyo_prefix..current_lidar
		-- Test the character device to see if it exists
		if os.execute("test -c "..device)==0 then
			local in_use = false
			-- TODO: Use fuser
			--local in_use = os.execute"test (fuser "..device.." )"==0
			if not in_use then
				found_device = device
			end
		end
	end
	assert( found_device, 'Did not find a Hokuyo' )
	-- Increment the current lidar
	libHokuyo.CURRENT_HOKUYO = current_lidar
	return self.new_hokuyo( found_device, serial, ttybaud )
end

---------------------------
-- Service multiple hokuyos
libHokuyo.new_hokuyo = function(ttyname, serial, ttybaud )
	local name = ttyname
	local baud = ttybaud or 115200

	-----------
	-- Open the Serial device with the proper settings
	--io.write( 'Opening ',name,' at ', baud, ' baud')
	local fd = unix.open( name, unix.O_RDWR + unix.O_NOCTTY)
	assert( fd>2, string.format("Open: %s, (%d)\n", name, fd)	)
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)
	assert( fd>2, string.format("stty parameter: %s, (%d)\n", name, fd)	)
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
	-- Setup the Hokuyo properly
	-----------
	set_baudrate(obj, baud)
	obj.params = get_sensor_params(obj)
	obj.info = get_sensor_info(obj)
	-----------

	-----------
	-- Set the methods for accessing the data
	-----------
	obj.stream_on = stream_on
	obj.stream_off = stream_off
	obj.get_scan = get_scan
	obj.callback = nil
	-- TODO: Use sensor_params.scan_rate
	obj.update_time = 1/40
	-----------

	-----------
	-- Return the hokuyo object
	return obj
	-----------
end

---------------------------
-- Service multiple hokuyos
-- TODO: This seems pretty generic already - make it more so
libHokuyo.service = function( hokuyos, main )

	local t0 = unix.time()
	-- Start the streaming of each hokuyo
	local projected_timestamps = {}
	assert(#objs==#callbacks,'Bad callback setup')
	local who_to_service = nil
	local t_future = math.huge
	for i,o in ipairs(objs) do
		local t_now = unix.time()
		local t_to_wait = o:stream_on()
		local future_time = t_now + o.update_time
		if future_time<t_future then
			who_to_service = o
			t_future = future_time
		end
		projected_timestamps[i] = future_time
	end

	-- TODO: Perform a main loop here?

	-- Loop and sleep appropriately
	while true do

		-- Sleep until ready to service
		local t_now = unix.time()
		local t_sleep = 1e6*(t_future-t_now)
		-- TODO: Debug if negative
		if t_sleep>0 then
			unix.usleep( t_sleep )
		end

		-- Service the correct Hokuyo
		local result = who_to_service:get_scan()
		if type( result ) == 'string' then
			-- Process the callback
			who_to_service.callback( result )
			-- Expect a new scan result
			projected_timestamps[i] = t_now + who_to_service.update_time
		else
			-- Update the next timestamp
			t_future = math.huge
			local mindex = -1
			for i,pt in ipairs(projected_timestamps) do
				if pt<t_future then
					mindex = i
					t_future = future_time
				end
			end
			-- Setup the correct hokuyo
			assert(mindex>0,'No object slated for next update!')
			who_to_service = objs[mindex]
		end

		-- Execute a main loop if desired, during the sleeping time
		if main then
			main()
		end

	end

end

return libHokuyo
