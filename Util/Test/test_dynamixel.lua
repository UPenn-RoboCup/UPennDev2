#!/usr/local/bin/luajit -i
dofile'../../include.lua'
-- Libraries
lD = require'libDynamixel'
vector = require'vector'
local ptable = require'util'.ptable
byte_to_number = lD.byte_to_number
nx_registers = lD.nx_registers
mx_registers = lD.mx_registers

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

function pStatus(status)
	if not status then return end
	--ptable(status)
	status = status[1]
	if not status then return end
	--ptable(status)
	local parse = byte_to_number[#status.parameter]
	if not parse then return end
	return parse(unpack(status.parameter)), status
end

--local found_ids = right_arm:ping_probe()
dofile'../../fiddle.lua'
