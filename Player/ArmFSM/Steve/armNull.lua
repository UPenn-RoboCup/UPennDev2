local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local t_entry, t_update, t_finish
local timeout = 10.0

-- Stiction.current offset (synonomous for now)
local STICTION_THRESHOLD = 10
-- Count the current offset
local stiction, n_stiction
-- Count the directions
local sample_dir, n_sample_dir
-- Count the inflections
local n_inflections, is_inflected
-- Count the steady state
local n_steady_state
-- Detect the direction and current offset and the joint initially
local d0, cur0, q0
-- Check if moving
local moving

-- The null degree of freedom
-- TODO: Use a jacbian and fun stuff
-- For now, just use the free parameter of the IK, which is just the third joint
local dof = 3

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t
	-- Determining stiction, etc.
	stiction = 0
	n_stiction = 0
  --
	sample_dir = 0
	n_sample_dir = 0
  --
	n_inflections = 0
  is_inflected = false
  --
	n_steady_state = 0
  --
	cur0 = false
	d0 = false
	q0 = Body.get_rarm_command_position()[dof]
  --
  moving = false
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  if t-t_entry > timeout then return'timeout' end

	-- Grab our data
	local cur = Body.get_rarm_current()[dof]
	local q = Body.get_rarm_position()[dof]

	-- Estimate the stiction while in this limit
	if moving==false then
    if math.abs(q-q0)*RAD_TO_DEG<0.02 then
  		-- Need 100 samples
  		if n_stiction < 100 then
  			n_stiction = n_stiction + 1
  			stiction = stiction + cur
  		end
  		-- Position is still close enough
  		return
    end
	end

	-- See if we have started moving
	if moving==false then
		if math.abs(cur-cur0) > STICTION_THRESHOLD then
			-- Check if away from the stiction significantly
			moving = true
      print('Moving the arm!')
      cur0 = stiction / n_stiction
      print('Offset current', cur0)
		else
			return
		end
	end

	local dCur = cur - cur0
	-- Check the direction of the current
	local d = util.sign(dCur)

	if not d0 then
		sample_dir = sample_dir + d
		n_sample_dir = n_sample_dir + 1
		if n_sample_dir>4 then
			d0 = util.sign(sample_dir)
			-- Keep going if no common direction
			if d0==0 then return end
			print('Set the Current direction', d0)
		else
			return
		end
	end

	if d~=d0 and is_inflected==false then
		n_inflections = n_inflections + (d==d0 and -1 or 1)
		n_inflections = math.max(0, n_inflections)
		if n_inflections > 5 then
			is_inflected = true
			print('Inflection Current direction!')
		end
	end

	local dq = q-q0
	-- Don't return on inflection. Wait until steady state
	if math.abs(dq)*RAD_TO_DEG < 0.04 then
		n_steady_state = n_steady_state + 1
	else
		n_steady_state = 0
	end
	if n_steady_state > 40 then
		print('Steady state')
		return'null'
	end

	-- Must only use our initial direction to react to the human
	-- Can just re-enter the state quickly on direction change
	if is_inflected or d~=d0 then return end

	print('dCur', dCur)
	-- Current opposes the direction
	local qCurGain = 1/7500
	local dqFromCur = qCurGain * dCur
	print('dq*', dqFromCur*RAD_TO_DEG, 'dq', dq*RAD_TO_DEG)
	local q1 = q0 - dqFromCur
  print('q0', q0*RAD_TO_DEG)
	print('q1', q1*RAD_TO_DEG, 'q', q*RAD_TO_DEG)
	----[[
	q0 = q1
	Body.set_rarm_command_position(q0, dof)
	--]]
	print()

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
