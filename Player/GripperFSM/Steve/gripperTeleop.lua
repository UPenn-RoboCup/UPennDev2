local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

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
  io.write(state._NAME..' Entry\n' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Reset the human desire
	local tqLGrip = Body.get_lgrip_command_torque()
	local tqRGrip = Body.get_rgrip_command_torque()
	hcm.set_teleop_lgrip_torque(tqLGrip)
  hcm.set_teleop_rgrip_torque(tqRGrip)
	-- NOTE: Assume torque here...? Must track this in hcm maybe
	--[[
	local qLGrip = Body.get_lgrip_position()
	local qRGrip = Body.get_rgrip_position()
	hcm.set_teleop_lgrip_position(qLGrip)
  hcm.set_teleop_rgrip_position(qRGrip)
	hcm.set_teleop_lgrip_mode(0)
  hcm.set_teleop_rgrip_mode(0)
	--]]

end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t 

  -- Grab our current position
	local qLGrip = Body.get_lgrip_position()
	local qRGrip = Body.get_rgrip_position()

	-- Teleop torque
	local tqL = hcm.get_teleop_lgrip_torque()
	if hcm.get_teleop_lgrip_mode()==1 then
		-- Override to use position control
		tqL = get_torque_requirement(qLGrip, hcm.get_teleop_lgrip_position())
	end

	local tqR = hcm.get_teleop_rgrip_torque()
	if hcm.get_teleop_rgrip_mode()==1 then
		-- Override to use position control
		tqR = get_torque_requirement(qRGrip, hcm.get_teleop_rgrip_position())
	end

	-- Set the torques
	Body.set_lgrip_command_torque(tqL)
	Body.set_rgrip_command_torque(tqR)
  
end

function state.exit()  
  io.write(state._NAME..' Exit\n' )
end

return state
