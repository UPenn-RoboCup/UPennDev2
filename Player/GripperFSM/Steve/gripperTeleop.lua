local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

local function get_torque_requirement(qGrip, qDesired, tqDesired)
	local tq = 0 * qGrip
	for i, q in ipairs(qGrip) do
		tq[i] = util.sign(q - qDesired[i]) * -tqDesired[i]
	end
	return tq
end

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Reset the human desire
	local qLGrip = Body.get_lgrip_position()
	local qRGrip = Body.get_rgrip_position()
	hcm.set_teleop_lgrip_position(qLGrip)
  hcm.set_teleop_rgrip_position(qRGrip)
	hcm.set_teleop_lgrip_torque(0)
  hcm.set_teleop_rgrip_torque(0)
	hcm.set_teleop_lgrip_mode(0)
  hcm.set_teleop_rgrip_mode(0)

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

	-- Find the torque to go to the zero position
  local tqL = get_torque_requirement(qLGrip)
	local tqR = get_torque_requirement(qRGrip)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
