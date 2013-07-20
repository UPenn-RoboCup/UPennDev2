-- libHokuyo
-- (c) 2013 Stephen McGill, Yida Zhang
-- Hokuyo Library

local libHokuyo = {}
local HokuyoPacket = require 'HokuyoPacket'
local stty = require 'stty'
local unix = require 'unix'

--------------------
-- libHokuyo constants
local HOKUYO_SCAN_REGULAR = 0
local HOKUYO_2DIGITS = 2
local HOKUYO_3DIGITS = 3

-- Coroutine read yielding
local N_SCAN_BYTES = 3372
-- Regular read
local FAST_READ_THRESHOLD = 3000 -- bytes
local SLOW_WAIT_US = 12000  -- (uses usleep)
local FAST_WAIT_SEC = 0.05 -- (uses select)
-- For the coroutine
local PREDICT_SLACK = 1.5e-3 -- Data comes in slightly earlier

--------------------
-- Internal paramters
libHokuyo.CURRENT_HOKUYO = -1
libHokuyo.MAX_NUM_HOKUYO = 255

-------------------------
-- Write a command to a hokuyo
local write_command = function(fd, cmd, expected_response)
	-- Should get one of these... '00', '02', '03', '04'
	expected_response = expected_response or '00'

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
		-- Ensure we get a prompt response
		-- TODO: Append here?
		local t_diff = unix.time() - t_write
		assert( t_diff<response_timeout, 
		string.format('Response timeout for command (%s)',cmd)
		)
		-- Read from the Hokuyo
		-- Sleep for each loop in wait of the response
		unix.usleep(5000)
		response = unix.read(fd)
		if type(response)=='string' then
			iCmd = response:find(cmd)
		end
	end
	--------------------

	--------------------
	-- Check the correctness of the response
	-- Just checking the last byte for now
	-- Filter to only the response
	response = response:sub(iCmd+#cmd)
	-- Soft fail
	if response:byte(#response)~=10 then
		print(string.format('Bad end of response!'))
		return nil
	end
	-- Last byte is 10 (\n)
	--[[
	assert(response:byte(#response)==10,
	string.format('Bad end of response! %s',response) 
	)
	]]
	--------------------

	--------------------
	-- Check the response
	local actual = response:sub(1,2)
	assert(
	actual==expected_response,
	string.format('Cmd (%s)Expected response (%s) but got (%s)', 
	cmd, expected_response, actual )
	)
	--------------------

	return response
end

-------------------------
-- Form the stream command
local create_scan_request = 
function(scan_start, scan_end, scan_skip, encoding, scan_type, num_scans)
-- TODO range checks and more types support
local request = nil
local base_request = string.format(
"%04d%04d%02x0%02d\n", scan_start, scan_end, scan_skip, num_scans
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

---------------------------
-- Grab the scan from the buffer
-- TODO: Add a timeout
-- NOTE: This is actually a coroutine
local get_scan = function(self)
	local buf = ''
	while true do
		unix.select({self.fd},FAST_WAIT_SEC)
		local raw_scan = unix.read( self.fd, N_SCAN_BYTES )
		if raw_scan then
			buf = buf..raw_scan
			if #buf==N_SCAN_BYTES then
				break
			end
		end
	end
	return HokuyoPacket.parse(buf)
end

-------------------------
-- Turn off data streaming
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

-------------------------
-- Turn on data streaming
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

-------------------------
-- Get sensor parameters from the hokuyo
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

-------------------------
-- Get Hokuyo sensor information
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
	
	-- Check if opened correctly
	--assert( fd>2, string.format("Open: %s, (%d)\n", name, fd)	)
	-- Use soft fail
	if fd<3 then
		return nil
	end
	
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
	for _,hokuyo in pairs(hokuyos) do
		print('Setting up',hokuyo.info.serial_number)
		io.flush()
		local t_now = unix.time()
		local t_to_wait = hokuyo:stream_on()
		local future_time = t_now + hokuyo.update_time
		--[[
		if future_time<t_future then
			who_to_service = hokuyo
			t_future = future_time
		end
		--]]
		hokuyo.t_last = t_now
		hokuyo.deadline = future_time
		hokuyo.thread = coroutine.create( 
		function()
			while true do
				local buf = ''
				local nbuf = 0
				repeat
					local raw_scan = unix.read(hokuyo.fd, N_SCAN_BYTES-nbuf )
					if raw_scan then
						buf = buf..raw_scan
						nbuf = #buf
					end
					--print('nbuf_intra_select',nbuf,unix.time())
					coroutine.yield( FAST_WAIT_SEC )
				until nbuf>=N_SCAN_BYTES
				-- Return the string to be parsed
				coroutine.yield( buf )
			end --while true
			end -- coroutine function
		)
	end

  local main_thread = nil
  if main then
    print('create main thread!')
    main_thread = coroutine.create( main )
  end

	-- Loop and sleep appropriately
	local who_to_service_id = 1
	local who_to_service = hokuyos[who_to_service_id]
	local t_future = who_to_service.deadline
	while true do

		-- Sleep until ready to service
		local t_now = unix.time()
		local t_sleep = 1e6*(t_future-t_now)
		if t_sleep>0 then
      print(t_sleep)
      if t_sleep>0.01 and main_thread then
        --print('Running main!')
        coroutine.resume( main_thread )
      end
			unix.usleep( t_sleep )
		end
		
		-- Resume the thread
		--print('Servicing',who_to_service.info.serial_number)
		local status_code, param = coroutine.resume( who_to_service.thread )
		
		-- Process the yielded result
		t_now = unix.time()
		if status_code then
			-- Process the result of the scan
			if type(param)=='string' then
				local t_diff = t_now-who_to_service.t_last
				who_to_service.t_last = t_now
				
				-- Process the callback
				if who_to_service.callback then
					local pkt = HokuyoPacket.parse(param)
					--print(who_to_service.info.serial_number,1/t_diff..' FPS')
					who_to_service.callback( pkt )
					--print()
				end
				-- Expect a new scan result
				who_to_service.deadline = t_now + who_to_service.update_time - PREDICT_SLACK
			else
				-- Quickly update
				unix.select({who_to_service.fd},param)
				who_to_service.deadline = t_now
			end
		else
			print('Dead hokuyo!',who_to_service.info.serial_number)
			table.remove(hokuyos,who_to_service_id)
		end
		
		-- Update the next timestamp
		t_future = math.huge
		who_to_service = nil
		who_to_service_id = -1
		for i,hokuyo in pairs( hokuyos ) do
			local d = hokuyo.deadline
			if d<t_future then
				who_to_service = hokuyo
				who_to_service_id = i
				t_future = d
			end
		end
		
		-- Setup the correct hokuyo
		assert(who_to_service,'No hokuyo slated for next update!')
	end -- while

end

return libHokuyo
