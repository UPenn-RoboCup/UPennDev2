local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

local procFunc = require'util'.procFunc
local deadPosition = 5*DEG_TO_RAD
local maxPosition = 30*DEG_TO_RAD
local maxTorque = 10
local tqGain = maxTorque / maxPosition
local minTorque = deadPosition * tqGain
local function get_torque_requirement(qGrip)
	local tq = {}
	for i, q in ipairs(qGrip) do
		tq[i] = procFunc(-tqGain * q, minTorque, maxTorque)
	end
	return tq
end


function state.entry()
  io.write(state._NAME..' Entry\n')
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	-- Grab our current position
	local qLGrip = Body.get_lgrip_position()
	local qRGrip = Body.get_rgrip_position()

	-- Find the torque to go to the zero position
  local tqL = get_torque_requirement(qLGrip)
	local tqR = get_torque_requirement(qRGrip)

	-- Write the torque
	Body.set_lgrip_command_torque(tqL)
	Body.set_rgrip_command_torque(tqR)

end

function state.exit()
  io.write(state._NAME..' Exit\n')
	-- Save our settings
	-- Try to retain the position of 0
	hcm.set_teleop_lgrip_position({0,0,0})
  hcm.set_teleop_rgrip_position({0,0,0})
	hcm.set_teleop_lgrip_mode(1)
  hcm.set_teleop_rgrip_mode(1)
end

return state
