dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local libDynamixel = require'libDynamixel'
local util = require'util'

if OPERATING_SYSTEM=='darwin' then
  right_arm = libDynamixel.new_bus('/dev/cu.usbserial-FTT3ABW9A')
  left_arm  = libDynamixel.new_bus'/dev/cu.usbserial-FTT3ABW9B'
else
  right_arm = libDynamixel.new_bus'/dev/ttyUSB0'
  left_arm  = libDynamixel.new_bus'/dev/ttyUSB1'
end

for i,m in ipairs({10,14}) do
  local status = libDynamixel.get_nx_mode(m,left_arm)
  if status then
    local value = libDynamixel.byte_to_number[#status.parameter](unpack(status.parameter))
    print(string.format('Mode: %d',value))
    if not (value==4) then
      print('setting',m)
      libDynamixel.set_nx_mode(m,4,left_arm)
    else
      print(m,'already set!')
    end
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
  end
end

