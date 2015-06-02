dofile'../../include.lua'
-- Libraries
local unix = require 'unix'
local lD = require'libDynamixel'

if not one_chain then
	if OPERATING_SYSTEM=='darwin' then
		chain = lD.new_bus()
	else
		rarm = lD.new_bus(Config.chain.rarm.ttyname)
		larm  = lD.new_bus(Config.chain.larm.ttyname)
		rleg = lD.new_bus(Config.chain.rleg.ttyname)
		lleg  = lD.new_bus(Config.chain.lleg.ttyname)
	end
end

local lleg_ids = Config.chain.lleg.m_ids
local rleg_ids = Config.chain.rleg.m_ids
local larm_ids = Config.chain.larm.m_ids
local rarm_ids = Config.chain.rarm.m_ids

local chain_ids = {lleg_ids, rleg_ids, larm_ids, rarm_ids}
local chains = {lleg, rleg, larm, rarm}
local names = {'lleg', 'rleg', 'larm', 'rarm'}
local indirects = {
	{'position', 'temperature', 'data', 'command_position', 'position_p'},
	{'position', 'temperature', 'data', 'command_position', 'position_p'},
	{'position', 'temperature', 'data', 'command_position', 'position_p'},
	{'position', 'temperature', 'data', 'command_position', 'position_p'}
}

-- Identifiers
local is_gripper = {}
for _,id in ipairs(Config.parts.LGrip) do is_gripper[id] = true end
for _,id in ipairs(Config.parts.RGrip) do is_gripper[id] = true end
local m_to_j, j_to_m = Config.servo.motor_to_joint, Config.servo.joint_to_motor
local max_rad = Config.servo.max_rad
local min_rad = Config.servo.min_rad

for i, ids in ipairs(chain_ids) do

	print('Checking', names[i], unpack(ids))
	local chain = chains[i]
	-- First, ping verify, to see the firmware

	print('Verifying...', chain:ping_verify(ids))

	-- Status return level to 1
	for _, id in ipairs(ids) do
		local jid = m_to_j[id]
		if chain.has_mx_cmd_id[id] then
			local status = lD.get_mx_status_return_level(id, self)
			local level = (type(status)=='table') and lD.parse_mx('status_return_level', status)
			if type(level)=='number' and level~=1 then
				print('Updating MX status_return_level', id)
				lD.set_mx_status_return_level(id, 1, chain)
			end
		elseif chain.has_nx_cmd_id[id] then
			local status = lD.get_nx_status_return_level(id, self)
			local level = (type(status)=='table') and lD.parse('status_return_level', status)
			if type(level)=='number' and level~=1 then
				print('Updating NX status_return_level', id)
				lD.set_nx_status_return_level(id, 1, chain)
			end
		end
	end

	-- Next, set the return delay
	for _, id in ipairs(ids) do
		local jid = m_to_j[id]
		if chain.has_mx_cmd_id[id] then
			local status = lD.get_mx_return_delay_time(id, self)
			local delay = (type(status)=='table') and lD.parse_mx('return_delay_time', status)
			if type(delay)=='number' and delay~=0 then
				print('Updating MX return_delay_time', id)
				lD.set_mx_return_delay_time(id, 0, chain)
			end
		elseif chain.has_nx_cmd_id[id] then
			local status = lD.get_nx_return_delay_time(id, self)
			local delay = (type(status)=='table') and lD.parse('return_delay_time', status)
			if type(delay)=='number' and delay~=0 then
				print('Updating NX return_delay_time', id)
				lD.set_nx_return_delay_time(id, 0, chain)
			end
		end
	end

	-- Now, Check the indirect addressing
	if false then
		local indirect = indirects[i]
		local indirect_ok = lD.check_indirect_address(ids, indirect, chain)
		if not indirect_ok then
			print('Updating indirect addressing')
			lD.set_indirect_address(ids, indirect, chain)
		end
	else
		print('Skipping indirect - please take off any MX motors before doing indrect!!')
	end

	-- Set the modes
	for _, id in ipairs(ids) do
		local jid = m_to_j[id]
		if is_gripper[jid] and chain.has_mx_cmd_id[id] then
			local status = lD.get_mx_alarm_shutdown(id, self)
			local alarm = (type(status)=='table') and lD.parse_mx('alarm_shutdown', status)
			-- Zero alarams :P
			if type(alarm)=='number' and alarm~=0 then
				print('Updating MX alarm_shutdown', id)
				lD.set_mx_alarm_shutdown(id, 0, chain)
			end
		elseif id==37 and chain.has_mx_cmd_id[id] then
			-- lidar
			local status = lD.get_mx_mode(id, self)
			local mode = type(status)=='table' and lD.parse_mx('mode', status)
		elseif chain.has_nx_cmd_id[id] and
			min_rad[jid]==-180*DEG_TO_RAD and max_rad[jid]==180*DEG_TO_RAD then
			-- 4
			local status = lD.get_nx_mode(id, self)
			local alarm = (type(status)=='table') and lD.parse_nx('mode', status)
			if type(alarm)=='number' and alarm~=4 then
				print('Updating NX mode', id, 4)
				lD.set_nx_mode(id, 4, chain)
			end
			-- Also set the shutdown
			local status = lD.get_nx_alarm_shutdown(id, self)
			local alarm = (type(status)=='table') and lD.parse_nx('alarm_shutdown', status)
			print('NX Alarm', id, jid, ':', alarm)
			if type(alarm)=='number' and alarm~=0 then
				print('Updating NX alarm_shutdown', id)
				lD.set_nx_alarm_shutdown(id, 0, chain)
			end
		elseif chain.has_nx_cmd_id[id] then
			-- 4
			local status = lD.get_nx_mode(id, self)
			local alarm = (type(status)=='table') and lD.parse_nx('mode', status)
			if type(alarm)=='number' and alarm~=3 then
				print('Updating NX mode', id, 3)
				lD.set_nx_mode(id, 3, chain)
			end
		end
		-- end of mode loop
	end

	-- end of chain loop
end
