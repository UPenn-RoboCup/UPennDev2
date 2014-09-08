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
		right_arm = lD.new_bus('/dev/ttyUSB0')
		left_arm  = lD.new_bus'/dev/ttyUSB1'
		right_leg = lD.new_bus'/dev/ttyUSB2'
		left_leg  = lD.new_bus'/dev/ttyUSB3'
	end
end

lD.set_nx_led_red(1,127,chain)
lD.set_nx_led_red(20,127,chain)
--
local ffi = require'ffi'
local left_ft = {
	id = Config.left_ft.id,
	m_ids = Config.left_ft.m_ids,
	raw = ffi.new'int16_t[4]',
	readings = ffi.new'double[6]',
	component = ffi.new'double[6]',
	--unloaded = ffi.new('double[6]',{1.13707, 1.31597, 0.984762, 0.813919, 1.24505, 1.50132}),
	--unloaded = ffi.new('double[6]',{1.243, 1.511, 1.343, 1.483, 1.144, 1.332}),
	--unloaded = ffi.new('double[6]', vector.zeros(6)),
	unloaded = ffi.new('double[6]', {1.227, 1.519, 1.342, 1.501, 1.142, 1.332}),
	calibration_mat = ffi.new('double[6][6]', Config.left_ft.matrix),
	calibration_gain = Config.left_ft.gain,
}
local function parse_ft(ft, raw_str, m_id)
	-- Lower ID has the 2 components
	if m_id==ft.m_ids[1] then
		ffi.copy(ft.raw, raw_str, 8)
		ft.component[0] = 3.3 * ft.raw[0] / 4095 - ft.unloaded[0]
		ft.component[1] = 3.3 * ft.raw[1] / 4095 - ft.unloaded[1]
		ft.component[2] = 3.3 * ft.raw[2] / 4095 - ft.unloaded[2]
		ft.component[3] = 3.3 * ft.raw[3] / 4095 - ft.unloaded[3]
		local v = vector.zeros(4)
		for i=0,#v-1 do v[i+1] = 3.3 * ft.raw[i] / 4095 end
		--print('A',v)
	elseif m_id==ft.m_ids[2] then
		ffi.copy(ft.raw, raw_str, 8)
		ft.component[4] = 3.3 * ft.raw[0] / 4095 - ft.unloaded[4]
		ft.component[5] = 3.3 * ft.raw[1] / 4095 - ft.unloaded[5]
		local v = vector.zeros(4)
		for i=0,#v-1 do v[i+1] = 3.3 * ft.raw[i] / 4095 end
		--print('B',v)
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
				* ft.calibration_gain
		end
	end
	return vector.slice(ft.readings, 0, 5), vector.slice(ft.component, 0, 5)
	--ffi.copy(ft.shm, ft.readings, ffi.sizeof(ft.readings))
	--if ft.id:find'217' then print(ft.id, vector.slice(ft.readings, 0, 5)) end
end

local pData = byte_to_number[nx_registers.data[2]]
local status, data, volt, proc
--for i=1,5 do
while true do
	status = lD.get_nx_data(1, chain)[1]
	if status then proc, volt = parse_ft(left_ft, status.raw_parameter, 26) end
	status = lD.get_nx_data(20, chain)[1]
	if status then proc, volt = parse_ft(left_ft, status.raw_parameter, 24) end
	--print(string.format("V: %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f", unpack(volt)))
	--print(string.format("P: %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f", unpack(proc)))
	print(string.format("Fx: %3.3f Fy: %3.3f Fz: %3.3f | Tz: %3.3f Ty: %3.3f Tx: %3.3f", unpack(proc)))
	unix.usleep(1e6)
end
os.exit()

local pTorqueMode = byte_to_number[mx_registers.torque_mode[2]]
local pTorque = byte_to_number[mx_registers.command_torque[2]]
local pVoltage = byte_to_number[mx_registers.min_voltage[2]]

local status = lD.get_mx_torque_mode(66, right_arm)
print('ID status', status)
if status then
	ptable(status)
	status = status[1]
	if status then
		ptable(status)
		print('Current mode', pTorqueMode(unpack(status.parameter)))
	end
end
local status = lD.set_mx_torque_enable(66, 1, right_arm)
local status = lD.set_mx_command_torque(66, 10, right_arm)
if status then
	ptable(status)
	status = status[1]
	if status then
		ptable(status)
	end
end
local status = lD.set_mx_torque_mode(66, 1, right_arm)
if status then
	ptable(status)
	status = status[1]
	if status then
		ptable(status)
	end
end
local status = lD.set_mx_command_torque(66, 10, right_arm)
if status then
	ptable(status)
	status = status[1]
	if status then
		ptable(status)
	end
end

--local found_ids = right_arm:ping_probe()
--[[
local status = lD.get_mx_id(66, right_arm)
print('ID status', status)
if status then ptable(status) end
if status[1] then ptable(status[1]) end
local status = lD.set_mx_led(66, 1, right_arm)
print('LED set status', status)
if status then ptable(status) end
if status[1] then ptable(status[1]) end
local status = lD.set_mx_led(67, 1, right_arm)
print('LED set status', status)
if status then ptable(status) end
if status[1] then ptable(status[1]) end

print()
local status = lD.get_mx_min_voltage(66, right_arm)
if status then
ptable(status)
if status[1] then
ptable(status[1])
print('min voltage', pVoltage(unpack(status[1].parameter))/10)
end
end
print()
local bad_limit = false
local status = lD.get_mx_max_voltage(66, right_arm)
if status then
ptable(status)
if status[1] then
ptable(status[1])
local v = pVoltage(unpack(status[1].parameter))/10
print('max voltage', v)
if v<24 then bad_limit = true end
end
end

if bad_limit then
print('UPDATE THE LIMIT')
status = nil
--local status = lD.set_mx_max_voltage(66, 220, right_arm)
if status then
ptable(status)
if status[1] then
ptable(status[1])
end
end
end

print()
local status = lD.get_mx_voltage(66, right_arm)
if status then
ptable(status)
if status[1] then
ptable(status[1])
print('Current voltage', pVoltage(unpack(status[1].parameter))/10)
end
end
--]]
--lD.set_indirect_address(found_ids, {'position', 'data'}, right_leg)
