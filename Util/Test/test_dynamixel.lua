dofile'../../include.lua'
-- Libraries
local lD = require'libDynamixel'
local util = require'util'

local p_instruction = lD.get_bulk(
  string.char(unpack({29, 30, 37})),
  { lD.nx_registers.position,
	  lD.nx_registers.position,
	  lD.mx_registers.position,
  }
)
print('Position Bulk')
local hex, dec = lD.tostring(p_instruction)
print(hex)
print()
print(dec)
print()

--os.exit()

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
-- Get the positions
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local p_parse_mx = lD.byte_to_number[lD.mx_registers.position[2]]

-- Bulk read testing
right_arm:ping_probe()
print("done")
local read_items, read_ids = {}, {}
for _, id in ipairs(right_arm.m_ids) do
	if right_arm.has_mx_id[id] then
		table.insert(read_items, lD.mx_registers.position)
		table.insert(read_ids, id)
  else
		table.insert(read_items, lD.nx_registers.position)
		table.insert(read_ids, id)
	end
end

ret = unix.write(right_arm.fd, p_instruction)
print("RET", ret)
status, ready = unix.select({right_arm.fd})
print('status',status, unpack(ready))

os.exit()

--[[
local statuses = lD.get_bulk(string.char(unpack(read_ids)), read_items, right_arm)
if type(statuses)=='table' then
  print('Got', #statuses)
  util.ptable(statuses)
  local val
  for _, status in pairs(statuses) do
	  util.ptable(status)
    if status.id==37 then
      val = p_parse_mx(unpack(status.parameter))
    else
      val = p_parse(unpack(status.parameter))
    end
    print("** VALUE=", val)
  end
elseif type(statuses)=='string' then
  print(statuses:byte(1, -1))
else
  print('statuses', statuses)
end
--]]

-- Set torque enable
--[[
lD.set_bulk(
	string.char(unpack({29, 30, 37})),
	{ lD.nx_registers.torque_enable,
		lD.nx_registers.torque_enable,
		lD.mx_registers.torque_enable,
	},
	{ 0,
		0,
		0,
	},
  right_arm
)
--]]

--[[
-- Set Command position - BE CAREFUL
local cp_instruction = lD.set_bulk(
	string.char(unpack({29, 30, 37})),
	{ lD.nx_registers.command_position,
		lD.nx_registers.command_position,
		lD.mx_registers.command_position,
	},
	{ 0,
		0,
		2048,
	},
  right_arm
)
--]]
