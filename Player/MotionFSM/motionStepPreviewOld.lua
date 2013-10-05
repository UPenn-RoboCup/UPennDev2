local state = {}
state._NAME = ...

local Body = require'Body'
local libZMP = require'libPreviewZMP'
local t_entry, t_update, t_finish
local timeout = 10.0

-- Walk params
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local footY = Config.walk.footY
--

-- Calculate the preview upon require
local zmp = libZMP.new_solver({
  tStep = Config.walk.tStep,
  tZMP  = Config.walk.tZMP,
  start_phase  = Config.walk.phSingle[1],
  finish_phase = Config.walk.phSingle[2]
})
-- Compute the preview matrices
zmp:compute_preview()

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Calculate the preview
  -- NOTE: only if params change...

  -- Generate the step queue, via hcm
  local step_seq = {}
  -- left foot step
  table.insert(step_seq, {0, {0.060,0,0}, {0,0}, 0.5})
  -- stop with feet together
  table.insert(step_seq, {2, {0,0,0},{0,0},4})
  -- Initialize the queue
  local uLeftI  = {-supportX,footY,0}
  local uRightI = {-supportX,-footY,0}
  local uTorsoI = {0,0,0}
  zmp:generate_step_queue(step_seq,uLeftI,uRightI)
  -- Initialize the solver
  zmp:init_preview(uTorsoI,uLeftI,uRightI,t_entry)

end

function state.update()
--  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  --repeat
    --t = t+preview_res
    local done = zmp:update_preview( t, supportX, supportY )
    zmp:solve_preview()
  --until t<=s.preview.clock or done

  -- Grab the desired center of mass
  local com = zmp:get_preview_com()
  print('CoM',com)

  if done then return 'done' end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state