dofile'../../include.lua'
-- Libraries
local lD = require'libDynamixel'
local util = require'util'.ptable

if not one_chain then
  if OPERATING_SYSTEM=='darwin' then
    right_arm = lD.new_bus('/dev/cu.usbserial-FTVTLUY0A')
    left_arm  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0B'
    right_leg = lD.new_bus'/dev/cu.usbserial-FTVTLUY0C'
    left_leg  = lD.new_bus'/dev/cu.usbserial-FTVTLUY0D'
  else
    right_arm = lD.new_bus('/dev/ttyUSB0')
    left_arm  = lD.new_bus'/dev/ttyUSB1'
    right_leg = lD.new_bus'/dev/ttyUSB2'
    left_leg  = lD.new_bus'/dev/ttyUSB3'
  end
end

local found_ids = left_leg:ping_probe()
libDynamixel.set_indirect_address(found_ids, {'position', 'data'}, left_leg)