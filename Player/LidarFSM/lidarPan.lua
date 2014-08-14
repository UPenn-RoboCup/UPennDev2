local state = {}
state._NAME = ...
local Body = require'Body'
local t_entry, t_update, t_exit
local min_pan, max_pan, mid_pan, mag_sweep
local t_sweep, ph, forward

-- Sync mesh parameters
local function update_pan_params()
	-- Necessary variables
	mag_sweep = 90 * DEG_TO_RAD
	t_sweep = 5
	-- Convenience variables
	min_pan = -mag_sweep/2
  max_pan = mag_sweep/2
  mid_pan = 0
end

function state.entry()
  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the updated pan paramters
  update_pan_params()
  
  -- Ascertain the phase, from the current position of the lidar
	if type(forward)~='boolean' or type(ph)~='number' then
  	local rad = Body.get_lidar_position()
		-- Take a given radian and back convert to find the current phase
		-- Direction is the side of the mid point, as a boolean (forward is true)
		-- Dir: forward is true, backward is false
		rad = math.max( math.min(rad, max_pan), min_pan )
  	ph, forward = ( rad - min_pan ) / mag_sweep, rad>mid_pan
		-- Check if we are *way* out of phase
		if ph>1.1 or ph<.1 then print('LIDAR WAY OUT OF PHASE') end
	end
	
	-- TODO: Send to the mesh channel our behavior
	
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
	local dt = t - t_update
	t_update = t
	
  -- Update the phase of the pan
	local is_forward = (forward and ph<1) or ph<=0
	ph = ph + (is_forward and 1 or -1) * (dt/t_sweep * mag_sweep)
	ph = math.max(math.min(ph, 1), 0)

  -- Set the desired angle of the lidar tilt
	Body.set_lidar_command_position(min_pan + ph * mag_sweep)
	
	-- We are switching directions, so emit an event
	if forward ~= is_forward then
		forward = is_forward
		ph = ph > 0.5 and 1 or 0
		return'switch'
	end
	
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
