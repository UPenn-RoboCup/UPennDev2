-- libHokuyo
-- (c) 2013 Stephen McGill, Yida Zhang
-- Hokuyo Library

local libHokuyo = {}
local HokuyoPacket = require'HokuyoPacket'
local stty = require'stty'
local unix = require'unix'
local tcp = require'tcp'

--------------------
-- libHokuyo constants
local HOKUYO_SCAN_REGULAR = 0
local HOKUYO_2DIGITS = 2
local HOKUYO_3DIGITS = 3

-- Reading properties
local N_SCAN_BYTES = 3372
local TIMEOUT = 0.05 -- (uses select)

-------------------------
-- Write a command to a hokuyo
local write_command = function(fd, cmd, expected_response)
	-- Should get one of these... '00', '02', '03', '04'
	expected_response = expected_response or '00'
  local response_timeout = 2
  
	--------------------
	-- Write the command
	local response = ''
	local ret = unix.write(fd, cmd)
  local t_write = unix.time()
  
  -- Attempt to find the correct response
  local iCmd = nil
	while not iCmd do
    
		-- Read from the Hokuyo
		-- Wait for the response
    local status, ready = unix.select( {fd}, TIMEOUT )
    -- Check the timeout
    if not status then return nil end
    if status>0 then
      local response_buf = unix.read(fd)
      response = response..response_buf
      iCmd = response:find(cmd)
    end
    
		-- Timeout if not finding the response in time
		local t_diff = unix.time() - t_write
    if t_diff>response_timeout then
      -- TODO: DEBUG should be error or print
  		--DEBUG( string.format('Response timeout for command (%s)',cmd) )
      return nil
    end
    
    end -- not iCmd
	--------------------

	--------------------
	-- Check the correctness of the response
	-- Just checking the last byte for now
	-- Filter to only the response
	response = response:sub(iCmd+#cmd)
	-- Last byte is 10 (\n)
	if response:byte(#response)~=10 then
		--DEBUG(string.format('Bad end of response!'))
		return nil
	end
	
	--------------------

	--------------------
	-- Check the response
  if response:sub(1,2)~=expected_response then
    --DEBUG(string.format('Cmd (%s)Expected response (%s) but got (%s)', 
    --cmd, expected_response, response:sub(1,2) ))
    return nil
  end
	--------------------

	return response
end

-------------------------
-- Form the stream command
-- TODO range checks and more types support
local create_scan_request = 
function(scan_start, scan_end, scan_skip, encoding, scan_type, num_scans)
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
local get_scan = function(self)
	local scan_str = ''
	while true do
		local status, ready = unix.select({self.fd},TIMEOUT)
    if not status then return nil end
		local scan_buf = unix.read( self.fd, N_SCAN_BYTES )
		if scan_buf then
			scan_str = scan_str..scan_buf
      if #scan_str==0 then
        local idx = scan_buf:find('99b')
        if idx then scan_str = scan_buf:sub(idx) end
      else
      	scan_str = scan_str..scan_buf -- Append it to the scan string
      end
      if #scan_str>=N_SCAN_BYTES then
				break
			end
		end
	end
	return HokuyoPacket.parse(scan_str)
end



-------------------------
-- Turn off data streaming
local stream_off = function(self)
	local ntries = 3
	local stop_delay = 5e4
	for i=1,ntries do
		local resp = write_command( self.fd, 'QT\n' )
		if resp then
			return true
		end
		-- TODO: Yield the delay time between stop attempts
		unix.usleep(stop_delay)
	end
  return false
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
	ret = write_command( self.fd, scan_req )

end

-------------------------
-- Get sensor parameters from the hokuyo
local get_sensor_params = function(self)
	local res = write_command( self.fd, 'PP\n' )

	local sensor_params = {}
	local params = HokuyoPacket.parse_info(res, 8)
  if not params then return nil end

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
  
	if not info then return nil end
  
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
-- Service multiple hokuyos over ethernet
libHokuyo.new_hokuyo_net = function(host, port)

	-- Open the tcp client
  local fd = tcp.open(host, port, 1)
  
	-----------
	-- Begin the Hokuyo object
	local obj = {}

	-----------
	-- Set the TCP data
	obj.fd = fd
	obj.host = host 
	obj.port = port
	obj.close = function(self)
		return unix.close(self.fd) == 0
	end
  
	-----------
	-- Set the methods for accessing the data
	obj.stream_on = stream_on
	obj.stream_off = stream_off
	obj.get_scan = get_scan
  obj.set_baudrate = set_baudrate
  obj.get_sensor_params = get_sensor_params
  obj.get_sensor_info = get_sensor_info
  obj.callback = nil
	-- TODO: Use sensor_params.scan_rate
	obj.update_time = 1/40
	-----------

	-----------
	-- Setup the Hokuyo properly
  if not obj:stream_off() then
    obj:close()
    return nil
  end
--	obj:set_baudrate(baud)
	obj.params = obj:get_sensor_params()
	obj.info = obj:get_sensor_info()
	-----------

	-----------
	-- Return the hokuyo object
	return obj
end


---------------------------
-- Service multiple hokuyos
libHokuyo.new_hokuyo = function(ttyname, serial, ttybaud )
  
  local baud = ttybaud or 115200
  
	if not ttyname then
		local ttys = unix.readdir("/dev");
		for i=1,#ttys do
			if ttys[i]:find("cu.usbmodem") or ttys[i]:find("ttyACM") then
				ttyname = "/dev/"..ttys[i]
        -- TODO: Test if in use
				break
			end
		end
	end

	-----------
	-- Open the Serial device with the proper settings
	local fd = unix.open( ttyname, unix.O_RDWR + unix.O_NOCTTY)
	-- Check if opened correctly
	if fd<3 then
    -- DEBUG(string.format("Open: %s, (%d)\n", name, fd))
		return nil
	end
  
  -----------
  -- Serial port settings
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)

	-----------
	-- Begin the Hokuyo object
	local obj = {}

	-----------
	-- Set the serial port data
	obj.fd = fd
	obj.ttyname = ttyname
	obj.baud = baud
	obj.close = function(self)
		return unix.close(self.fd) == 0
	end
  
	-----------
	-- Set the methods for accessing the data
	obj.stream_on = stream_on
	obj.stream_off = stream_off
	obj.get_scan = get_scan
  obj.set_baudrate = set_baudrate
  obj.get_sensor_params = get_sensor_params
  obj.get_sensor_info = get_sensor_info
  obj.callback = nil
	-- TODO: Use sensor_params.scan_rate
	obj.update_time = 1/40
	-----------

	-----------
	-- Setup the Hokuyo properly
  if not obj:stream_off() then
    obj:close()
    return nil
  end
	obj:set_baudrate(baud)
	obj.params = obj:get_sensor_params()
	obj.info = obj:get_sensor_info()
	-----------

	-----------
	-- Return the hokuyo object
	return obj
end

---------------------------
-- Service multiple hokuyos
-- TODO: This seems pretty generic already - make it more so
libHokuyo.service = function( hokuyos, main )
  
  -- Enable the main function as a coroutine thread
  local main_thread = nil
  if main then
    main_thread = coroutine.create( main )
  end

	-- Start the streaming of each hokuyo
  -- Instantiate the hokuyo coroutine thread
  local hokuyo_fds = {}
  local fd_to_hokuyo = {}
  local fd_to_hokuyo_id = {}
	for i,hokuyo in ipairs(hokuyos) do
    fd_to_hokuyo[hokuyo.fd] = hokuyo
    fd_to_hokuyo_id[hokuyo.fd] = i
    table.insert(hokuyo_fds,hokuyo.fd)
		hokuyo:stream_on()
		hokuyo.t_last = unix.time()
		hokuyo.thread = coroutine.create( 
		function()
      print('Starting coroutine for',hokuyo.info.serial_number)
      -- The coroutine should never end
      local scan_str = ''
			while true do -- extract buffer
        -- Grab the latest data from the hokuyo buffer
				local scan_buf = unix.read(hokuyo.fd, N_SCAN_BYTES-#scan_str )
        -- If no return, something maybe went awry with the hokuyo
        if not scan_buf then
          print('BAD READ',type(scan_buf),hokuyo.info.serial_number)
          return
        end
        if #scan_str==0 then
          -- This is where the start of the packet is
          local idx = scan_buf:find('99b')
          -- If we do not find the preamble of the scan, ignore read
          if idx then
            -- The scan string starts at the index of 99b
            -- TODO: Read the documentation for why
            scan_str = scan_buf:sub(idx)
          end
        else
          -- Append it to the scan string
					scan_str = scan_str..scan_buf
        end
        -- Check if we are done
        if #scan_str>=N_SCAN_BYTES then
    			-- Return the scan string to be parsed
          hokuyo.t_last = unix.time()
    			coroutine.yield( scan_str )
          scan_str = ''
        else
          -- Wait for the hokuyo buffer to fill again
          coroutine.yield()
        end
      end -- while extract buffer

		end -- coroutine function
		)
	end
  
  -- Loop while the hokuyos are alive
	while #hokuyos>0 do

    -- Perform Select on all hokuyos
    local status, ready = unix.select( hokuyo_fds )
    for i,is_ready in pairs(ready) do
      if is_ready then
        local who_to_service = fd_to_hokuyo[i]
        -- Resume the thread
        local status_code, param = coroutine.resume( who_to_service.thread )
        -- Check if there were errors in the coroutine
        if not status_code then
          print('Dead hokuyo coroutine!',who_to_service.info.serial_number)
          who_to_service:close()
          local h_id = fd_to_hokuyo_id[i]
          table.remove( hokuyos, h_id )
          table.remove( hokuyo_fds, h_id )
        end
        -- Process the callback
        if param and who_to_service.callback then
          who_to_service.callback( HokuyoPacket.parse(param) )
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

return libHokuyo
