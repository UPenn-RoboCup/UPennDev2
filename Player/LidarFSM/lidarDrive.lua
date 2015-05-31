local state = {}
state._NAME = ...
local Body = require'Body'
require'vcm'
local t_entry, t_update, t_exit
local min_pan, max_pan, mid_pan, mag_sweep
local t_sweep, ph, forward
local min, max = math.min, math.max

-- Sync mesh parameters
local function update_pan_params()
	-- Necessary variables
	mag_sweep, t_sweep = unpack(vcm.get_mesh0_sweep())
	-- Some simple safety checks
	mag_sweep = min(max(mag_sweep, 10 * DEG_TO_RAD), math.pi)
	t_sweep = min(max(t_sweep, 0.5), 10)
	-- Convenience variables
	min_pan = -mag_sweep/2
  max_pan = mag_sweep/2
  mid_pan = 0
end

function state.entry()
--  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
	-- Torque enable
	Body.set_lidar_torque_enable(1)
	Body.set_lidar_command_position(0)
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
	local dt = t - t_update
	t_update = t
end

function state.exit()
--  print(state._NAME..' Exit' )
end

return state
