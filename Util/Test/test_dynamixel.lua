dofile'../../include.lua'
-- Libraries
local lD = require'libDynamixel'
local util = require'util'

local function instruction_tostring(instruction)
	local instruction_bytes = {}
	local instruction_ints = {}
	for i, v in ipairs({instruction:byte(1,-1)}) do
		table.insert(instruction_bytes, string.format(' %02X', v))
		table.insert(instruction_ints, string.format('%3d', v))
	end
	return table.concat(instruction_bytes, ' '), table.concat(instruction_ints, ' ')
end

local p_instruction = lD.get_bulk(string.char(unpack({29, 30, 37})), {
		lD.nx_registers.position,
		lD.nx_registers.position,
		lD.mx_registers.position
	})

local cp_instruction = lD.set_bulk(
	string.char(unpack({29, 30, 37})),
	{ lD.nx_registers.command_position,
		lD.nx_registers.command_position,
		lD.mx_registers.command_position,
	},
	{ 0,
		0,
		2048,
	}
)
print('Position Bulk')
local hex, int = instruction_tostring(p_instruction)
print(hex)
print()
print(int)
print()
print('Command Position Bulk')
local hex, int = instruction_tostring(cp_instruction)
print(hex)
print()
print(int)

--os.exit()

--one_chain = libDynamixel.new_bus()

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
--    grippers  = lD.new_bus('/dev/ttyUSB4',1000000)
  end
end

-- Choose a chain
local bus = assert(right_leg, 'Bus does not exist')

-- Get the positions
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local p_parse_mx = lD.byte_to_number[lD.mx_registers.position[2]]
--[[
print('Ping Probe', bus.ttyname)
local found = bus:ping_probe()
print('Inspecting', table.concat(found, ','))
unix.usleep(1e4)
local positions = lD.get_nx_position(bus.m_ids, bus)
for _, status in pairs(positions) do
  local val = p_parse(unpack(status.parameter))
  print('==', val)
  util.ptable(status)
end
--]]

-- Bulk read testing
right_arm:ping_probe()
local read_items, read_ids = {}, {}
for _, id in ipairs(right_arm.m_ids) do
	if right_arm.has_mx_id[id] then
		table.insert(read_items, lD.mx_registers.position)
		table.insert(read_ids, id)
	elseif right_arm.has_nx_id[id] then
		table.insert(read_items, lD.nx_registers.position)
		table.insert(read_ids, id)
	end
end
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

-- Set torque enable
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
