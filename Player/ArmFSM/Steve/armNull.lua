local state = {}
state._NAME = ...

local torch = require'torch'
local Body = require'Body'
local K = require'K_ffi'
local util = require'util'
local t_entry, t_update, t_finish
local timeout = 10.0

-- Gain for moving the arm
local qCurGain = 1 / 7500
-- Count the current offset
local sample_cur
-- Count the directions
local sample_dir
-- Count the inflections
local n_inflections, is_inflected, was_inflected
-- Count the steady state
local n_steady_state
-- Detect the direction and current offset and the joint initially
local d0, cur0, q0
-- Check if moving
local moving
-- Threholds
local cur_std
--
local qErr

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
	--
	sample_cur = {}
	sample_dir = {}
  --
	n_inflections = 0
  is_inflected = false
  was_inflected = false
  --
	n_steady_state = 0
  --
	cur0 = false
	d0 = false
	q0 = Body.get_rarm_command_position()[dof]
  -- error in the position (sag)
  qErr = Body.get_rarm_position()[dof] - q0
  --
  cur_std = math.huge
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
  local qArm = Body.get_rarm_position()
	local cur = Body.get_rarm_current()[dof]
	local q = qArm[dof]

	-- Estimate the sample_cur while in this limit
	if moving==false then
    -- Need 100 samples
    if #sample_cur < 100 then
      if math.abs(q-q0)*RAD_TO_DEG<0.02 then
        table.insert(sample_cur, cur)
  		end
      -- After inserting, check
      if #sample_cur < 100 then return end
      -- Get the statistics
      local s = torch.Tensor(sample_cur)
      cur0 = s:mean()
      -- Assign the threshold
      cur_std = s:std()
    end
    -- We are moving if outside twice our std dev
		if math.abs(cur-cur0) < 2*cur_std then return end
		moving = true
    print('Moving the arm!')
	end

	local dCur = cur - cur0
  local dq = q - q0
	-- Check the direction of the current and position
	local ddCur = util.procFunc(dCur, cur_std, 1)
  local ddQ = util.procFunc(dq, cur_std, 1)

	if #sample_dir < 5 then
    table.insert(sample_dir, ddCur)
    d0 = util.sign(sample_dir)
    if #sample_dir < 5 or d0==0 then return end
    print('Set the Current direction', d0)
	end

  -- Check if an inflection occurred
	if ddCur~=d0 then
		n_inflections = n_inflections + 1
	else
		n_inflections = n_inflections - 1
		n_inflections = math.max(0, n_inflections)
	end
  is_inflected = n_inflections > 5
  if is_inflected~=was_inflected then
    print('Inflection change', is_inflected)
  end
  was_inflected = is_inflected

	-- Don't return on inflection. Wait until steady state
	if math.abs(dq)*RAD_TO_DEG < 0.05 then
		n_steady_state = n_steady_state + 1
	else
		n_steady_state = n_steady_state - 1
	end
	if n_steady_state > 40 then
		print('Steady state')
		return'null'
	end

	-- Must only use our initial direction to react to the human
	-- Can just re-enter the state quickly on direction change
	if ddCur~=d0 then
    print('outlier dir')
    return
  end

  -- Calculate the new position to use for the arm
	local dqFromCur = qCurGain * dCur
  local q1 = q0 - dqFromCur
  io.write('dCur', dCur, 'd0', d0, 'ddCur', ddCur, '\n')
	io.write('dq*', dqFromCur*RAD_TO_DEG, 'dq', dq*RAD_TO_DEG, '\n')
  io.write('q0', q0*RAD_TO_DEG, '\n')
	io.write('q1', q1*RAD_TO_DEG, 'q', q*RAD_TO_DEG, '\n')
  io.write('\n')

  -- Set on the robot
  -- NOTE: THIS CAN BE DANGEROUS!
	q0 = q1
	Body.set_rarm_command_position(q0, dof)

  --[[
  local trArm = K.forward_rarm(qArm)
  -- Get the inverse using the new free dof parameter (shoulderYaw)
  local iqArm = K.inverse_rarm(trArm, qArm, q0)
  Body.set_rarm_command_position(iqArm)
	--]]

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
