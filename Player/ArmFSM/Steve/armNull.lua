local state = {}
state._NAME = ...

local torch = require'torch'
local Body = require'Body'
local K = require'K_ffi'
local util = require'util'
local t_entry, t_update, t_finish
local timeout = 10.0

-- Count the current offset
local sample_cur
-- Count the directions
local sample_dir
-- Count the inflections
local n_inflections, is_inflected, was_inflected
-- Count the steady state
local n_steady_state
-- Detect the direction and current offset and the joint initially
local dir0, cur0, q0
-- Check if moving
local moving
-- Threholds
local cur_std
--
local qErr, qArm0, trArm0
local q_last, qc_last, cur_last
local zeroCur_last, zeroHappened
local start_count
--
local qSag0

-- The null degree of freedom
-- TODO: Use a jacbian and fun stuff
-- For now, just use the free parameter of the IK, which is just the third joint
local dof = 3
local t_move

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
	dir0 = false
	curArm0 = Body.get_rarm_current()
	qcArm0 = Body.get_rarm_command_position()
	qArm0 = Body.get_rarm_position()
  trArm0 = K.forward_rarm(qArm0)
	cur0 = curArm0[dof]
	q0 = qArm0[dof]
	qc0 = qcArm0[dof]
	q_last = q0
	qc_last = qc0
	cur_last = cur0
  -- error in the position (sag)
  qSag0 = q0 - qc0
  --
  cur_std = math.huge
  --
  moving = false
	zeroHappened = false
	--
	start_count = 0
	--print('Calibrating...', qSag0*RAD_TO_DEG)
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
  local curArm = Body.get_rarm_current()
  local qArm = Body.get_rarm_position()
  local qcArm = Body.get_rarm_command_position()
	--
	local cur = curArm[dof]
  local q = qArm[dof]
  local qc = qcArm[dof]
	--
	local dCur0 = cur - cur0
  local dq0 = q - q0
  local dqc0 = qc - qc0
	--
	local dCur = cur - cur_last
	local dq = q - q_last
	local dqc = qc - qc_last
	--
	q_last = q
	qc_last = qc
	cur_last = cur

	-- Estimate the sample_cur while in this limit
	if moving==false then
    -- Need 100 samples
    if #sample_cur < 100 then
			if math.abs(dq0)*RAD_TO_DEG > 0.02 then return'null' end
			table.insert(sample_cur, cur)
      -- After inserting, check
      if #sample_cur < 100 then return end
      -- Get the statistics
      local s = torch.Tensor(sample_cur)
      cur0 = s:mean()
      -- Assign the threshold
      cur_std = math.max(s:std(), 3)
			qSag0 = q0 - qc0
			--print('Calibrated', cur0, cur_std)
			--print('qSag0', qSag0*RAD_TO_DEG)
			-- hack for now
			t_entry = t
		end
		local dCurAbs0 = math.abs(dCur0)
		if dCurAbs0<2*cur_std then
			local beta = 0.6
			cur0 = cur0 * beta + cur * (1-beta)
		end
		if dCurAbs0 > 2*cur_std then
			start_count = start_count + 1
		else
			start_count = math.max(start_count - 1, 0)
			if start_count == 0 then sample_dir = {} end
		end
		if start_count<3 then return end
		local qSag = q - qc
		local dir = util.sign(qSag - qSag0)
		if #sample_dir < 5 then
			table.insert(sample_dir, dir)
			if #sample_dir < 5 then return end
			local s = torch.Tensor(sample_dir)
			dir0 = util.sign(s:sum())
			if dir0==0 then return end
		end
		moving = true
		zeroCur_last = cur
		print('Moving the arm', dir0, zeroCur_last)
		t_move = t
	end
	
	if dCur==0 then
		local dZeroCur = cur - zeroCur_last
		if util.sign(zeroCur_last) ~= util.sign(cur) then
			io.write('\ncur: ', cur)
			io.write('\ndZeroCur: ', dZeroCur, '\tzeroCur_last: ', zeroCur_last)
			io.write('\n')
			if t-t_move>1 then zeroHappened = true end
		end
		zeroCur_last = cur
	end
	
	if zeroHappened then print('zero occurred') return'null' end
	--print('dCur', dCur)
  if t-t_entry > 8 then return'null' end

  local qc_next = qc + dir0 * 1/10 * DEG_TO_RAD
	Body.set_rarm_command_position(qc_next, dof)

	if true then return end

	-- Check the direction of the current and position
	local dirCur = util.sign(util.procFunc(dCur0, cur_std, 1))
  local dirQ = util.sign(util.procFunc(dq, 0.02 * DEG_TO_RAD, 1))
	local dir = dirCur

  -- Need at least 5 points to sum
	if #sample_dir < 5 then
    table.insert(sample_dir, dirCur)
    if #sample_dir < 5 then return end
    local s = torch.Tensor(sample_dir)
    dir0 = util.sign(s:sum())
    if dir0==0 then return end
    print('dir0= '..dir0)
		qdir0 = util.sign(q - q0)
		print('qdir0', qdir0)
	end

	-- Don't return on inflection. Wait until steady state
	----[[
	--]]



	if zeroHappened then
		print('dq', math.abs(dq)*RAD_TO_DEG, dCur)
		if math.abs(dq)*RAD_TO_DEG < 0.02 then
			n_steady_state = n_steady_state + 1
		else
			n_steady_state = n_steady_state - 1
		end
		if n_steady_state > 25 then
			print('Steady state')
			return'null'
		end
		return
	end

--	if math.abs(dCur)>20 then return end

  local qc_next = qc - dir0 * 1/8 * DEG_TO_RAD
	--io.write('\nqc: ', qc*RAD_TO_DEG, '\tq_next: ', qc_next*RAD_TO_DEG)
	--io.write('\nq: ', q*RAD_TO_DEG, '\tq_last: ', q_last*RAD_TO_DEG)
	
	--if true then return'null' end

  -- Set on the robot
  -- NOTE: THIS CAN BE DANGEROUS!
	--[[
	q0 = q1
	Body.set_rarm_command_position(q, dof)
	--]]
	
	----[[
	Body.set_rarm_command_position(qc_next, dof)
	--]]

  --[[
	q0 = q1
  -- Get the inverse using the new free dof parameter (shoulderYaw)
  local iqArm = K.inverse_rarm(trArm0, qArm, q0)
  Body.set_rarm_command_position(iqArm)
	--]]
	

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
