local state = {}
state._NAME = ...

local Body = require'Body'
local vector = require'vector'
local t_entry, t_update

local teleopLGrip, teleopRGrip

local procFunc = require'util'.procFunc
local deadPosition = 5*DEG_TO_RAD
local maxPosition = 45*DEG_TO_RAD
local maxTorque = 20
local tqGain = maxTorque / maxPosition
local minTorque = deadPosition * tqGain
local function get_torque_requirement(qGrip, qDesired)
	local tq = {}
	for i, q in ipairs(qGrip) do
		local dq = q - qDesired[i]
		table.insert(tq, procFunc(-tqGain * dq, minTorque, maxTorque))
	end
	return tq
end

function state.entry()
  print(state._NAME..' Entry\n' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Set where we are
	local tqLGrip = Body.get_lgrip_command_torque()
	local tqRGrip = Body.get_rgrip_command_torque()
	teleopLGrip = tqLGrip
	teleopRGrip = tqRGrip
	hcm.set_teleop_lgrip_torque(teleopLGrip)
  hcm.set_teleop_rgrip_torque(teleopRGrip)

end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	-- Check for changes
	local teleopLGrip1 = hcm.get_teleop_lgrip_torque()
	local teleopRGrip1 = hcm.get_teleop_rgrip_torque()
	local lChange = teleopLGrip1~=teleopLGrip
	local rChange = teleopRGrip1~=teleopRGrip

		if lChange then
		teleopLGrip = teleopLGrip1
		Body.set_lgrip_command_torque(teleopLGrip)
	end

	if rChange then
		teleopRGrip = teleopRGrip1
		Body.set_rgrip_command_torque(teleopRGrip)
	end

end

function state.exit()
  print(state._NAME..' Exit\n' )
end

return state
