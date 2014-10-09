dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local lD = require'libDynamixel'
local util = require'util'


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

local byte_to_number = lD.byte_to_number
local nx_registers = lD.nx_registers
local mx_registers = lD.mx_registers

local lleg = Config.chain.lleg
local lleg_ok = lD.check_indirect_address(lleg.m_ids, {'position', 'data'}, left_leg)
print('LLeg Check', lleg_ok)
if not lleg_ok then
  lD.set_indirect_address(lleg.m_ids, {'position', 'data'}, left_leg)
end

local rleg = Config.chain.rleg
local rleg_ok = lD.check_indirect_address(rleg.m_ids, {'position', 'data'}, right_leg)
print('RLeg Check', rleg_ok)
if not rleg_ok then
  lD.set_indirect_address(rleg.m_ids, {'position', 'data'}, right_leg)
end

os.exit()

for i,m in ipairs(Config.chain.lleg) do
	local status = libDynamixel.check_indirect_address(m, left_leg)
	if status then
		local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
		print(string.format('Mode: %d',value))
		if not (value==4) then
			print('setting',m)
			libDynamixel.set_nx_mode(m,4,left_arm)
		else
			print(m,'already set!')
		end
	else
		print('MOTOR',m,'not responding')
	end
end

for i,m in ipairs({9,13}) do
	local status = libDynamixel.get_nx_mode(m,right_arm)
	if status then
		local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
		print(string.format('Mode: %d',value))
		if not (value==4) then
			print('setting',m)
			libDynamixel.set_nx_mode(m,4,right_arm)
		else
			print(m,'already set!')
		end
	else
		print('MOTOR',m,'not responding')
	end
end

