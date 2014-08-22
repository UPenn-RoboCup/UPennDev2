-- libMicrostrain
-- (c) 2013 Stephen McGill
-- Microstrain Library

local libMicrostrain = {}
local stty = require'stty'
local unix = require'unix'
local ffi = require'ffi'
local bit = require'bit'

--for k,v in ipairs(response) do print(string.format('%d: %02X',k,v)) end
local function cmd2string(cmd, do_print)
  local instruction_bytes = {}
  local instruction_decs = {}
  for i, v in ipairs(cmd) do
    table.insert(instruction_bytes, string.format(' %02X', v))
    table.insert(instruction_decs, string.format('%03d', v))
  end
  local hex, dec = table.concat(instruction_bytes, ' '), table.concat(instruction_decs, ' ')
  if do_print then
    print("Packet Length:", #cmd)
    print('hex', hex)
    print('dec', dec)
  end
  return hex, dec
end

-- Checksum formation
local function generate_packet(byte_array)
  local checksum_byte1, checksum_byte2 = 0, 0
  for _,val in ipairs(byte_array) do
    checksum_byte1 = (checksum_byte1 + val) % 256
    checksum_byte2 = (checksum_byte2 + checksum_byte1) % 256
  end
  table.insert(byte_array,checksum_byte1)
  table.insert(byte_array,checksum_byte2)
  -- Check the length
  if #byte_array-6 ~= byte_array[4] then
    print("BAD PAYLOAD LENGTH", #byte_array-6, byte_array[4])
    cmd2string(byte_array, true)
    os.exit()
  end
  return string.char(unpack(byte_array))
end

-- Write a command to a microstrain
local function write_command(fd, cmd)
  local str = generate_packet(cmd)
  local ret = unix.write(fd, str)
  assert(ret==#str, 'Bad write of command!')
  local status, ready = unix.select( {fd}, 0.1 )
  assert(status>0,'Timeout! '..status)
  local res = unix.read(fd)
  assert(res, 'No data!')
  if not res then return end
  return {res:byte(1,-1)}
end

local function idle(dev)
  write_command(dev.fd, { 0x75, 0x65, 0x01, 0x02, 0x02, 0x02 })
end

-- Model number, etc.
local function get_info(self)
  local response = write_command(self.fd, {0x75, 0x65, 0x01, 0x02, 0x02, 0x03})
  local pkt1_idx = 5
  local pkt1_sz  = response[5]
  local pkt2_idx = 5+pkt1_sz --4+pkt1_sz+1
  local pk2_sz   = response[pkt2_idx]
  local firmware_version = 256 * response[pkt2_idx+2]+response[pkt2_idx+3]
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
  idle(microstrain)

  -- Write the command
  --local response = write_command(microstrain.fd,baud_cmd)
  --for k,v in ipairs(response) do print(string.format('%d: %02X',k,v)) end
  --microstrain:close()
  --unix.usleep(1e6)
  -- Open with new baud
  --libMicrostrain.new_microstrain(microstrain.ttyname,baud,microstrain)

  -- Ping the microstrain
  write_command(microstrain.fd, { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01 })
end

local function enable_magnetometer_compensation(microstrain)
  local DISABLE_MAG = true
  -- Disables magnetometer and north
  local data_conditioning_flags = bit.bor( 0x0100, 0x0400)
  -- Disables only north
  --local data_conditioning_flags = 0x0400
  local hex_flags = bit.tohex(data_conditioning_flags, 4):gmatch('%d%d')
  local flag_bytes = {}
  local disable_north_compensation_cmd = { 0x75, 0x65, 0x0C,
    0x10,
    0x10, 0x35,
    0x01, -- Apply new settings
    0x00, 0x0A, -- default decimation
  }
  if DISABLE_MAG then
    for hex in hex_flags do
      table.insert(disable_north_compensation_cmd, tonumber(hex, 16) )
    end
  else
    table.insert(disable_north_compensation_cmd, 0x04)
    table.insert(disable_north_compensation_cmd, 0x00)
  end
  for _, v in ipairs(
  {
    0x0E, -- New Accel/Gyro Filter Width
    0x11, -- New Mag Filter Width
    0x00, 0x0A, -- New Up Compensation
    0x00, 0x0A, -- New North Compensation
    0x01, -- New Mag Bandwidth/Power
    0x00, 0x00, -- Reserved
  })
  do
    table.insert(disable_north_compensation_cmd, v)
  end
  --cmd2string(disable_north_compensation_cmd, true)
  -- Send to the device
  write_command(microstrain.fd, disable_north_compensation_cmd)
end

local function configure(self, do_permanent)
  -- Set the mode for reading data
  -- 100Hz of gyro & rpy

  -- New AHRS format
  local stream_fmt = { 0x75, 0x65, 0x0C,
    0x13, -- Command length
    0x13, 0x08, -- Field Length, and Field Desctiption (AHRS)
    0x01, 0x05, -- Set 5 messages
    0x04, 0x00, 0x01, -- Accel Scaled Message @ 100Hz
    0x05, 0x00, 0x01, -- Gyro Scaled Message @ 100Hz
    0x07, 0x00, 0x01, -- Delta Theta Message @ 100Hz
    0x0C, 0x00, 0x01, -- Euler Angles Message @ 100Hz
    0x06, 0x00, 0x01, -- Magnetometer Message @ 100Hz
  }
  local response = write_command(self.fd, stream_fmt)
  unix.usleep(1e5)

  -- New NAV format
  stream_fmt = { 0x75, 0x65, 0x0C,
    0x0D, -- Command length
    0x0D, 0x0A, -- Field Length, and Field Desctiption (NAV)
    0x01, 0x03, -- Set 3 messages
    0x10, 0x00, 0x01, -- status
    0x05, 0x00, 0x01, -- Estimated Orientation, Euler Angles @ 100Hz
    0x06, 0x00, 0x01, -- Estimated Gyro Bias
  }
  local response = write_command(self.fd, stream_fmt)
  
  -- Just AHRS
  local save_fmt = { 0x75, 0x65, 0x0C,
    0x04, -- Command length
    0x04, 0x08, -- Packet length
    0x03, 0x00 -- 3 to perform the save
  }
  
  -- AHRS and NAV
--[[
  local save_fmt = { 0x75, 0x65, 0x0C,
    0x08, -- Command length
    0x04, 0x08, -- Packet length
    0x03, 0x00, -- 3 to perform the save
    0x04, 0x0A, -- Packet length
    0x03, 0x00 -- 3 to perform the save
  }
--]]
  local response = write_command(self.fd,save_fmt)

  -- Set the initial heading to zero
  local init_heading = { 0x75, 0x65, 0x0D,
    0x06, -- Command length
    0x06, 0x03, -- Packet length
    0x00, 0x00,
    0x00, 0x00,
  }
  local response = write_command(self.fd, init_heading)

  -- Set the initial attitude
  local init_att = { 0x75, 0x65, 0x0D,
    0x06, -- Command length
    0x06, 0x04, -- Packet length
    0x00, 0x00,
    0x00, 0x00,
  }
  local response = write_command(self.fd, init_att)

  if do_permanent then
    -- Device startup settings
  end
end

local function close(self)
  idle(self)
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
end

local function ahrs_and_nav_on(self)
  -- Turn on the ahrs stream
  local response = write_command(self.fd, {
    0x75, 0x65, 0x0C, 0x0A,
    0x05, 0x11, 0x01, 0x01, 0x01, -- ahrs
    0x05, 0x11, 0x01, 0x03, 0x01, -- nav
  })
end

-- TODO: Make this like input_co of libDynamixel
local acc_tmp, gyr_tmp, mag_tmp, euler_tmp, del_gyr_tmp=
	ffi.new'float[3]', ffi.new'float[3]', ffi.new'float[3]', ffi.new'float[3]', ffi.new'float[3]'
local nav_stat = ffi.new"uint16_t[3]"
local cpy_sz = 3 * ffi.sizeof('float')

local function read_ahrs(self)
  local fd = self.fd
  unix.select({fd})
  local buf = unix.read(fd)
  if not buf then return end
	-- Try to select some stuff
	-- Accel
	ffi.copy(acc_tmp, buf:sub(7, 18):reverse(), cpy_sz)
	-- Gyro
	ffi.copy(gyr_tmp, buf:sub(21, 32):reverse(), cpy_sz)
	-- Delta
	ffi.copy(del_gyr_tmp, buf:sub(35, 46):reverse(), cpy_sz)
	-- Euler
	ffi.copy(euler_tmp, buf:sub(49, 60):reverse(), cpy_sz)
	-- Mag
	ffi.copy(mag_tmp, buf:sub(63, 74):reverse(), cpy_sz)

  -- NAV stuff
--[[
  if #buf>76 then
    -- Debugging
--    cmd2string({buf:byte(77, -1)}, true)
    ffi.copy(nav_stat, buf:sub(83, 88):reverse(), 3 * ffi.sizeof('uint16_t'))
  end
--]]

  --[[
	-- Using the non-FFI API
  local _gyro = carray.float(buf:sub( 7,18):reverse())
  local _rpy  = carray.float(buf:sub(21,32):reverse())
  --]]

  return acc_tmp, gyr_tmp, del_gyr_tmp, euler_tmp, mag_tmp
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
	assert(fd>2, string.format("Open: %s, (%d)\n", ttyname, fd))

  -- Serial port settings
	stty.raw(fd)
	stty.serial(fd)
	stty.speed(fd, baud)
  unix.usleep(1e4)

	local dev = {
    fd = fd,
    ttyname = ttyname,
    baud = baud,
    idle = idle,
    close = close,
    ahrs_on = ahrs_on,
    ahrs_off = ahrs_off,
    ahrs_and_nav_on = ahrs_and_nav_on,
    get_info = get_info,
    read_ahrs = read_ahrs,
  }

  -- Set the device to idle
  idle(dev)
  unix.usleep(1e5)
  
  -- Configure params
  enable_magnetometer_compensation(dev)
  
	-- Configure the device
	configure(dev)

  return dev

end

return libMicrostrain
