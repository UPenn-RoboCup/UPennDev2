local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

local function get_torque_requirement(qGrip, tqDesired, qDesired)
	if not qDesired then return tqDesired end
	local tq = 0 * qGrip
	for i, q in ipairs(qGrip) do
		tq[i] = util.sign(q - qDesired[i]) * -tqDesired[i]
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

	-- Find the torque
	local tqL = get_torque_requirement(
		qLGrip,
		tqL,
		hcm.get_teleop_lgrip_mode()==1 and hcm.get_teleop_lgrip_position()
	)

	-- Set the torques
	Body.set_lgrip_command_torque(tqL)
  
end

function state.exit()  
  io.write(state._NAME..' Exit\n' )
end

return state
