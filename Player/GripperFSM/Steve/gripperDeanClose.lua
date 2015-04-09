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

local qLGrip0 = Config.demo.arms.dean.qLGrip
local qRGrip0 = Config.demo.arms.dean.qRGrip

function state.entry()
  io.write(state._NAME, ' Entry\n')
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
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

	local tqL = get_torque_requirement(qLGrip, qLGrip0)
	local tqR = get_torque_requirement(qRGrip, qRGrip0)

	-- Set the torques
	Body.set_lgrip_command_torque({-5,-5,tqL[3]})
	Body.set_rgrip_command_torque({5,tqR[2],-5})

end

function state.exit()
  io.write(state._NAME, ' Exit\n')
end

return state
