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
		chain = right_leg
	end
end

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
