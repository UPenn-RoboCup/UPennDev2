dofile'../../include.lua'
-- Libraries
local lD = require'libDynamixel'
local util = require'util'

--one_chain = libDynamixel.new_bus()

if not one_chain then
  if OPERATING_SYSTEM=='darwin' then
    right_arm = lD.new_bus('/dev/cu.usbserial-FTVTLUY0A')
    left_arm  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0B'
    right_leg = lD.new_bus'/dev/cu.usbserial-FTVTLUY0C'
    left_leg  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0D'
  else
    right_arm = lD.new_bus('/dev/ttyUSB0', 4.5e6 --[[38400]])
    left_arm  = lD.new_bus'/dev/ttyUSB1'
    right_leg = lD.new_bus'/dev/ttyUSB2'
    left_leg  = lD.new_bus'/dev/ttyUSB3'
--    grippers  = lD.new_bus('/dev/ttyUSB4',1000000)
  end
end

-- Choose a chain
local bus = assert(right_arm, 'Bus does not exist')
print('Ping Probe', bus.ttyname)
local found = bus:ping_probe(2)
print('Inspecting', table.concat(found, ','))
unix.usleep(1e4)

-- Which Motor ID to inspect
local m_id, reg = 30, 'baud_rate'
local parse = lD.byte_to_number[lD.nx_registers[reg][2]]
local s = lD['get_nx_'..reg](m_id, bus)
local status = assert(s[1], 'No status!')
local value = parse(unpack(status.parameter))
print('ID', m_id, reg, value)
unix.usleep(1e4)

-- Set the baud to 4 Mbps
--[[
print('Update the baud rate')
local s = lD['set_nx_baud_rate'](m_id, 6, bus)
local status = assert(s[1], 'No status!')
util.ptable(status)
--]]
