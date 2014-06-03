-- libMicrostrain
-- (c) 2013 Stephen McGill
-- Microstrain Library

local libMicrostrain = {}
local stty = require'stty'
local unix = require'unix'

-- Reading properties
local TIMEOUT = 0.05 -- (uses select)
local ping_cmd = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01 }
local idle_cmd = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x02 }

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
local function write_command(fd, cmd)
  local str = generate_packet(cmd)
  local ret = unix.write(fd,str)
  assert(ret==#str,'Bad save write!')
  local fd_id = unix.select( {fd}, TIMEOUT )
  assert(fd_id==1,'Timeout!')
  local res = unix.read(fd)
  assert(res,'No data!')
  return {res:byte(1,-1)}
end

local function get_info(self)
  local response = write_command(self.fd, {0x75, 0x65, 0x01, 0x02, 0x02, 0x03})
  -- Parse information
  local pkt1_idx = 5
  local pkt1_sz  = response[5]
  local pkt2_idx = 5+pkt1_sz --4+pkt1_sz+1
  local pk2_sz   = response[pkt2_idx]
  local firmware_version = 256*response[pkt2_idx+2]+response[pkt2_idx+3]
  local information = string.char(unpack(response,pkt2_idx+4,pkt2_idx+pk2_sz-1))
  local info = {firmware_version}
  for k in information:gmatch('[^%s]+') do table.insert(info,k) end
  return info
end

-- Go to high speed baud
local function change_baud(microstrain)
  local baud = 921600 -- 921600 only for now...
  local baud_cmd = { 0x75, 0x65, 0x0C,
    0x07, -- Command length
    0x07, 0x40, -- Length and command description
    0x01, -- USE this setting (not saved at boot, tho...)
    0x00, 0x0E, 0x10, 0x00
  }

  -- Set the device to idle
  local response = write_command(microstrain.fd,idle_cmd)

  -- Write the command
  --local response = write_command(microstrain.fd,baud_cmd)
  --for k,v in ipairs(response) do print(string.format('%d: %02X',k,v)) end
  --microstrain:close()
  --unix.usleep(1e6)
  -- Open with new baud
  --libMicrostrain.new_microstrain(microstrain.ttyname,baud,microstrain)

  -- Ping the microstrain
  local response = write_command(microstrain.fd,ping_cmd)
  --[[
  for k,v in ipairs(response) do
    print(string.format('%d: %02X',k,v))
  end
  --]]
end

local function configure(self, do_permanent)
  -- Set the mode for reading data
  -- 100Hz of gyro & rpy

  -- New AHRS format
  local stream_fmt = { 0x75, 0x65, 0x0C,
    0x10, -- Command length
    0x10, 0x08, -- Field Length, and Field Desctiption (AHRS)
    0x01, 0x04, -- Set 4 messages
    0x04, 0x00, 0x01, -- Accel Scaled Message @ 100Hz
    0x05, 0x00, 0x01, -- Gyro Scaled Message @ 100Hz
    0x06, 0x00, 0x01, -- Magnetometer Message @ 100Hz
    0x0C, 0x00, 0x01, -- Euler Angles Message @ 100Hz
  }
  local response = write_command(self.fd,stream_fmt)
  --[[
  for k,v in ipairs(response) do
    print(string.format('%d: %02X',k,v))
  end
  --]]

  if do_permanent then
    -- Save only once! Maybe in the eeprom, so lots of saving could be bad...
    local save_fmt = { 0x75, 0x65, 0x0C,
      0x04, -- Command length
      0x04, 0x08, -- Packet length
      0x03, 0x00 -- 3 to perform the save
    }
    local response = write_command(self.fd,save_fmt)
    --[[
    for k,v in ipairs(response) do
      print(string.format('%d: %02X',k,v))
    end
    --]]
  end
end

local function close(self)
  return unix.close(self.fd) == 0
end

local function ahrs_on(self)
  -- Turn on the ahrs stream
  local response = write_command(self.fd, {
    0x75, 0x65, 0x0C, 0x05, 0x05, 0x11, 0x01, 0x01, 0x01
  })
end
local function ahrs_off(self)
  -- Turn off the ahrs stream
  local response = write_command(self.fd, {
    0x75, 0x65, 0x0C, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00
  })
  --for i,b in ipairs(response) do print( string.format('%d: %02X',i,b) ) end
end

---------------------------
-- Service multiple microstrains
function libMicrostrain.new_microstrain(ttyname, ttybaud)

  local baud = ttybaud or 115200

	if not ttyname then
		local ttys = unix.readdir("/dev")
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
	local fd = unix.open(ttyname, unix.O_RDWR + unix.O_NOCTTY)
	-- Check if opened correctly
	if fd<3 then
    print(string.format("Open: %s, (%d)\n", ttyname, fd))
		return nil
	end

  -- Serial port settings
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)
  unix.usleep(1e5)

  -- Set the device to idle
  write_command(fd, idle_cmd)

	-----------
	-- Yield the Microstrain object
	return {
    fd = fd,
    ttyname = ttyname,
    baud = baud,
    configure = configure,
    close = close,
    ahrs_on = ahrs_on,
    ahrs_off = ahrs_off,
    get_info = get_info,
  }

end

---------------------------
-- Service multiple microstrains
-- TODO: This seems pretty generic already - make it more so
function libMicrostrain.service(microstrain, main)
  -- Ensure a callback
  assert(type(microstrain.callback)=='function','Need a callback!')

  -- Go as fast as possible
  microstrain:change_baud()

  -- Configure to have the right format
  microstrain:configure()

  -- Enable the main function as a coroutine thread
  local main_thread
  if type(main)=='function' then
    main_thread = coroutine.create( main )
  end

  local thread = coroutine.create(
    function()
      while true do
        local ret = unix.select({microstrain.fd})
        res = unix.read(microstrain.fd)
        assert(res,'Bad response!'..type(ret))
        coroutine.yield(
          carray.float(res:sub(7,18):reverse()), -- gyro
          carray.float(res:sub(21,32):reverse()) -- rpy
          )
      end
    end
  )
  assert(thread,'Could not create thread')
  --print('thread!',thread,coroutine.status(thread))

  -- Turn on streaming
  microstrain:ahrs_on()

  -- Loop while the microstrain is alive
	repeat
    local status, ready = unix.select( {microstrain.fd} )
    local status_code, acc, gyr = coroutine.resume( thread )
    -- Check if there were errors in the coroutine
    if status_code then
      microstrain.callback(acc,gyr)
    else
      print( 'Dead microstrain coroutine!', acc)
      microstrain:ahrs_off()
      microstrain:close()
    end
    -- Resume the main thread
    if main_thread then coroutine.resume( main_thread ) end
  until not status_code

end

return libMicrostrain
