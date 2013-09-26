-- libMicrostrain
-- (c) 2013 Stephen McGill, Yida Zhang
-- Microstrain Library

local libMicrostrain = {}
--local MicrostrainPacket = require'MicrostrainPacket'
local stty = require'stty'
local unix = require'unix'

-- Reading properties
local TIMEOUT = 0.05 -- (uses select)

-- Checksum formation (slow, but speed is unneeded)
local function generate_packet(byte_array)
  local checksum_byte1, checksum_byte2 = 0, 0
  for _,val in ipairs(byte_array) do
    checksum_byte1 = (checksum_byte1 + val)%256
    checksum_byte2 = (checksum_byte2 + checksum_byte1)%256
  end
  table.insert(byte_array,checksum_byte1)
  table.insert(byte_array,checksum_byte2)
  return string.char( unpack(byte_array) )
end

-------------------------
-- Write a command to a microstrain
local function write_command(fd,cmd)
  local str = generate_packet(cmd)
  local ret = unix.write(fd,str)
  assert(ret==#str,'Bad save write!')
  local fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  local res = unix.read(fd)
  assert(res,'No data!')
  local response = {res:byte(1,-1)}
  --for i,b in ipairs(response) do print( string.format('%d: %02X',i,b) ) end
  return response
end

---------------------------
-- Service multiple microstrains
libMicrostrain.new_microstrain = function(ttyname, ttybaud )
  
  local baud = ttybaud or 115200
  
	if not ttyname then
		local ttys = unix.readdir("/dev");
		for _,tty in ipairs(ttys) do
			if tty:find("cu.usbmodem") or tty:find("ttyACM") then
				ttyname = "/dev/"..tty
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
    print(string.format("Open: %s, (%d)\n", name, fd))
		return nil
	end
  
  -----------
  -- Serial port settings
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)

  -----------
  -- Gut check: ping the device
  local ping_cmd = 
    string.char( 0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6)
  local ret = unix.write(fd,ping_cmd)
  assert(ret==#ping_cmd,'Bad ping write!')
  -- Wait until the device responds with data
  local fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  -- Grab the response
  local res = unix.read(fd)
  assert(res,'No data!')
  local response = {res:byte(1,-1)}
  --print('Ping Response:')
  --for i,b in ipairs(response) do print( string.format('%d: %X',i,b) ) end

  -- Set to idle
  local idle_cmd = 
    string.char( 0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7 )
  ret = unix.write(fd,idle_cmd)
  assert(ret==#idle_cmd,'Bad idle write!')
  fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  res = unix.read(fd)
  assert(res,'No data!')
  response = {res:byte(1,-1)}
  --print('Idle Response:')
  --for i,b in ipairs(response) do print( string.format('%d: %X',i,b) ) end

--7565 0102 0203 E2C8
  local info_cmd = 
    string.char( 0x75, 0x65, 0x01, 0x02, 0x02, 0x03, 0xE2, 0xC8 )
  ret = unix.write(fd,info_cmd)
  assert(ret==#info_cmd,'Bad info write!')
  fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  res = unix.read(fd)
  assert(res,'No data!')
  response = {res:byte(1,-1)}
  local pkt1_idx = 5
  local pkt1_sz  = response[5]
  local pkt2_idx = 5+pkt1_sz --4+pkt1_sz+1
  local pk2_sz   = response[pkt2_idx]
  local firmware_version = 256*response[pkt2_idx+2]+response[pkt2_idx+3]
  local information = string.char(unpack(response,pkt2_idx+4,pkt2_idx+pk2_sz-1))
  local info = {firmware_version}
  for k in information:gmatch('[^%s]+') do table.insert(info,k) end

  -- Set the mode for reading data
  -- 100Hz of gyro, imu, timestamp
  -- Copy/paste from the docs
  --[[
  local stream_fmt = { 0x75, 0x65, 0x0C, 0x0D, 0x0D, 0x08, 0x01, 0x03, 0x04, 0x00, 0x01, 0x05, 0x00, 0x01, 0x12, 0x00, 0x01}
  -- Make the checksum and yield the string
  local stream_fmt_cmd = generate_packet(stream_fmt)
  ret = unix.write(fd,stream_fmt_cmd)
  assert(ret==#stream_fmt_cmd,'Bad stream write!')
  fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  res = unix.read(fd)
  assert(res,'No data!')
  response = {res:byte(1,-1)}
  --for i,b in ipairs(response) do print( string.format('%d: %02X',i,b) ) end

  -- Save only once! Maybe in the eeprom, so lots of saving could be bad...
  local save_fmt = { 0x75, 0x65, 0x0C, 0x04, 0x04, 0x08, 0x03, 0x00 }
  -- Make the checksum and yield the string
  local save_fmt_cmd = generate_packet(save_fmt)
  ret = unix.write(fd,save_fmt_cmd)
  assert(ret==#save_fmt_cmd,'Bad save write!')
  fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  res = unix.read(fd)
  assert(res,'No data!')
  response = {res:byte(1,-1)}
  for i,b in ipairs(response) do print( string.format('%d: %02X',i,b) ) end
  --]]
  --]]

  -- TODO: Use NAV or not?? This is the EKF filtered stuff... quaternion format...

	-----------
	-- Begin the Microstrain object
	local obj = {}

	-----------
	-- Set the serial port data
	obj.fd = fd
  obj.info = info
	obj.ttyname = ttyname
	obj.baud = baud
	obj.close = function(self)
		return unix.close(self.fd) == 0
	end
  obj.ahrs_on = function(self)
    -- Turn on the ahrs stream
    local response = write_command(fd,{
      0x75, 0x65, 0x0C, 0x05, 0x05, 0x11, 0x01, 0x01, 0x01
    })
  end
  obj.ahrs_off = function(self)
    -- Turn off the ahrs stream
    local response = write_command(fd,{
      0x75, 0x65, 0x0C, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00
    })
    --for i,b in ipairs(response) do print( string.format('%d: %02X',i,b) ) end
  end

	-----------
	-- Return the microstrain object
	return obj
end

---------------------------
-- Service multiple microstrains
-- TODO: This seems pretty generic already - make it more so
libMicrostrain.service = function( microstrains, main )
  
  -- Enable the main function as a coroutine thread
  local main_thread = nil
  if main then
    main_thread = coroutine.create( main )
  end

	-- Start the streaming of each microstrain
  -- Instantiate the microstrain coroutine thread
  local microstrain_fds = {}
  local fd_to_microstrain = {}
  local fd_to_microstrain_id = {}
	for i,microstrain in ipairs(microstrains) do
    fd_to_microstrain[microstrain.fd] = microstrain
    fd_to_microstrain_id[microstrain.fd] = i
    table.insert(microstrain_fds,microstrain.fd)
		microstrain:stream_on()
		microstrain.t_last = unix.time()
		microstrain.thread = coroutine.create( 
		function()
      print('Starting coroutine for',microstrain.info.serial_number)
      -- The coroutine should never end
      local scan_str = ''
			while true do -- extract buffer
        -- Grab the latest data from the microstrain buffer
				local scan_buf = unix.read(microstrain.fd, N_SCAN_BYTES-#scan_str )
        -- If no return, something maybe went awry with the microstrain
        if not scan_buf then
          print('BAD READ',type(scan_buf),microstrain.info.serial_number)
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
          microstrain.t_last = unix.time()
    			coroutine.yield( scan_str )
          scan_str = ''
        else
          -- Wait for the microstrain buffer to fill again
          coroutine.yield()
        end
      end -- while extract buffer

		end -- coroutine function
		)
	end
  
  -- Loop while the microstrains are alive
	while #microstrains>0 do

    -- Perform Select on all microstrains
    local status, ready = unix.select( microstrain_fds )
    for i,is_ready in pairs(ready) do
      if is_ready then
        local who_to_service = fd_to_microstrain[i]
        -- Resume the thread
        local status_code, param = coroutine.resume( who_to_service.thread )
        -- Check if there were errors in the coroutine
        if not status_code then
          print('Dead microstrain coroutine!',who_to_service.info.serial_number)
          who_to_service:close()
          local h_id = fd_to_microstrain_id[i]
          table.remove( microstrains, h_id )
          table.remove( microstrain_fds, h_id )
        end
        -- Process the callback
        if param and who_to_service.callback then
          who_to_service.callback( MicrostrainPacket.parse(param) )
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

return libMicrostrain
