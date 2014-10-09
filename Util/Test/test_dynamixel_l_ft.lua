dofile'../../include.lua'
-- Libraries
local lD = require'libDynamixel'
local vector = require'vector'
local ptable = require'util'.ptable
local byte_to_number = lD.byte_to_number
local nx_registers = lD.nx_registers
local mx_registers = lD.mx_registers

if not one_chain then
	if OPERATING_SYSTEM=='darwin' then
		--[[
		right_arm = lD.new_bus('/dev/cu.usbserial-FTVTLUY0A')
		left_arm  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0B'
		right_leg = lD.new_bus'/dev/cu.usbserial-FTVTLUY0C'
		left_leg  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0D'
		--]]
		--chain = lD.new_bus(nil, 57600)
		chain = lD.new_bus()
	else
		left_leg = lD.new_bus'/dev/ttyUSB3'
		chain = left_leg
	end
end

--
local ffi = require'ffi'
local left_ft = {
  id = Config.left_ft.id,
  m_ids = Config.left_ft.m_ids,
  raw16 = ffi.new'int16_t[4]',
  raw = ffi.new'uint8_t[8]',
  readings = ffi.new'double[6]',
  component = ffi.new'double[6]',
--  unloaded = ffi.new('double[6]', vector.zeros(6)),
  unloaded = ffi.new('double[6]', Config.left_ft.unloaded),
  calibration_mat = ffi.new('double[6][6]', Config.left_ft.matrix),
  calibration_gain = Config.left_ft.gain,
}


local function parse_ft(ft, raw_str, m_id)
	-- Lower ID has the 2 components
	if m_id==ft.m_ids[1] then
		ffi.copy(ft.raw, raw_str, 8)
		local raw16_as_8 = ffi.cast('uint8_t*', ft.raw16)
		-- Endian swap
		for i=0,7,2 do
			--raw16_as_8[i] = ft.raw[i+1]
			--raw16_as_8[i+1] = ft.raw[i]
			raw16_as_8[i] = ft.raw[i]
			raw16_as_8[i+1] = ft.raw[i+1]
		end
		ft.component[0] = 3.3 * tonumber(ft.raw16[0]) / 4095.0 - ft.unloaded[0]
		ft.component[1] = 3.3 * tonumber(ft.raw16[1]) / 4095.0 - ft.unloaded[1]
		ft.component[2] = 3.3 * tonumber(ft.raw16[2]) / 4095.0 - ft.unloaded[2]
		ft.component[3] = 3.3 * tonumber(ft.raw16[3]) / 4095.0 - ft.unloaded[3]
		local v = vector.zeros(4)
		for i=0,#v-1 do v[i+1] = 3.3 * ft.raw16[i] / 4096 end
		--print(m_id,'A',v)
		for i=0,#v-1 do v[i+1] = ft.raw16[i] end
		--print(m_id,'A',v)
	elseif m_id==ft.m_ids[2] then
		ffi.copy(ft.raw, raw_str, 8)
		local raw16_as_8 = ffi.cast('uint8_t*', ft.raw16)
		-- Endian swap
		for i=0,7,2 do
			--raw16_as_8[i] = ft.raw[i+1]
			--raw16_as_8[i+1] = ft.raw[i]
			raw16_as_8[i] = ft.raw[i]
			raw16_as_8[i+1] = ft.raw[i+1]
		end
		ft.component[4] = 3.3 * tonumber(ft.raw16[0]) / 4095.0 - ft.unloaded[4]
		ft.component[5] = 3.3 * tonumber(ft.raw16[1]) / 4095.0 - ft.unloaded[5]
		local v = vector.zeros(4)
		for i=0,#v-1 do v[i+1] = 3.3 * ft.raw16[i] / 4096 end
		--print(m_id,'B',v)
		for i=0,#v-1 do v[i+1] = ft.raw16[i] end
		--print(m_id,'B',v)
	else
		return
	end
	--if ft.id:find'217' then print(ft.id, vector.slice(ft.component, 0, 5)) end
	-- New is always zeroed
	ffi.fill(ft.readings, ffi.sizeof(ft.readings))
	for i=0,5 do
		for j=0,5 do
			ft.readings[i] = ft.readings[i]
				+ ft.calibration_mat[i][j]
				* ft.component[j]
--				* ft.calibration_gain
		end
	end
	return vector.slice(ft.readings, 0, 5), vector.slice(ft.component, 0, 5), vector.slice(ft.raw, 0, 3)
	--ffi.copy(ft.shm, ft.readings, ffi.sizeof(ft.readings))
	--if ft.id:find'217' then print(ft.id, vector.slice(ft.readings, 0, 5)) end
end

local pData = byte_to_number[nx_registers.data[2]]
local status, data, volt, proc, rawA, rawB
--for i=1,5 do
while true do

  print('\n===')

	status = lD.get_nx_data(26, chain)[1]
	if status then proc, volt, rawA = parse_ft(left_ft, status.raw_parameter, 26) end
	status = lD.get_nx_data(24, chain)[1]
	if status then proc, volt, rawB = parse_ft(left_ft, status.raw_parameter, 24) end

--	print(string.format("A: %3.3f, %3.3f, %3.3f, %3.3f", unpack(rawA)))
--	print(string.format("B: %3.3f, %3.3f, %3.3f, %3.3f", unpack(rawB)))
--	print(string.format("V: %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f", unpack(volt)))
	print(string.format("Fx: %3.3f Fy: %3.3f Fz: %3.3f | Tx: %3.3f Ty: %3.3f Tz: %3.3f", unpack(proc)))
	unix.usleep(0.5*1e6)
end
os.exit()

