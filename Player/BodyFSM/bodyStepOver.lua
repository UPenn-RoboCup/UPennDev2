local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

require'mcm'

-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM',true)

local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local pose, target_pose
local step_planner
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg



local function calculate_footsteps()
  local step_queue



--For webots with 10ms
  if IS_WEBOTS then
    step_queue={
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.27,0,0},0,  0.25, 1,   0.5,   {0,-0.02,0}, {0,0.20,0.15}},   --LS
      {{0.27,0,0},1,  0.5,   1.1, 0.5,    {0,0.02,0},  {0,0.20,0.15}},    --RS
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.27,0,0} ,0, 0.5,   1.1, 0.5,    {0,-0.02,0},  {0.15,0.20,0.0}},--LS
      {{0.27,0,0}, 1, 0.5,  1.1, 0.5,   {0,0.02,0},  {0.15,0.20,0.0}},--RS
      {{0,0,0,},  2,   0.1, 2, 1,     {0,0.0,0},  {0, 0, 0}},                  --DS
    }
  else  
  step_queue={
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.27,0,0},0,  0.5, 2.2,   1,   {0,-0.01,0}, {0,0.20,0.142}},   --LS
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},

      {{0.27,0,0},1,  1,   2.2, 1,    {0.0,0.0,0},  {0,0.20,0.142}},    --RS

      {{0,0,0},   2,  0.1, 3, 0.1,   {0,0.0,0},  {0, 0, 0}},

      {{0.25,0,0} ,0, 1,   2.2,  1,    {0,0.0,0},  {0.142,0.20,0.0}},--LS
      {{0,0,0},   2,  0.1, 2, 0.1,   {0,0.0,0},  {0, 0, 0}},

      {{0.25,0,0}, 1,  1,  2.2, 1,   {0,0.0,0},  {0.142,0.20,0.0}},--RS



      {{0,0,0,},  2,   0.1, 2, 1,     {0,0.0,0},  {0, 0, 0}},                  --DS
    }
  end




--THRESHOLD WALKOVER MOTION

--For webots with 10ms
  if IS_WEBOTS then
    step_queue={
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.27,0,0},0,  0.25, 1,   0.5,   {0,-0.02,0}, {0,0.10,0.0}},   --LS
      {{0.27,0,0},1,  0.5,   1.1, 0.5,    {0,0.02,0},  {0,0.10,0.0}},    --RS
      {{0,0,0,},  2,   0.1, 2, 1,     {0,0.0,0},  {0, 0, 0}},                  --DS
    }
  else  
  step_queue={
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
--      {{0.27,0,0},0,  0.5, 2.2,   1,   {0,-0.01,0}, {0,0.10,0.0}},   --LS

      {{0.30,0,0},0,  0.5, 2.2,   1,   {0,-0.01,0}, {0,0.10,0.0}},   --LS


      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},

--      {{0.27,0,0},1,  1,   2.2, 1,    {0.0,0.0,0},  {0,0.10,0.0}},    --RS
      {{0.30,0,0},1,  1,   2.2, 1,    {0.0,0.0,0},  {0,0.10,0.0}},    --RS

      {{0,0,0,},  2,   0.1, 2, 1,     {0,0.0,0},  {0, 0, 0}},                  --DS
    }


--slightly longer wait between steps
  step_queue={
      {{0,0,0},   2,  0.1, 1, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.30,0,0},0,  0.5, 2.2,   1,   {0,-0.01,0}, {0,0.08,0.0}},   --LS
      {{0,0,0},   2,  0.1, 2.2, 0.1,   {0,0.0,0},  {0, 0, 0}},
      {{0.30,0,0},1,  1,   2.2, 1,    {0.0,0.0,0},  {0,0.10,0.0}},    --RS
      {{0,0,0,},  2,   0.1, 2, 1,     {0,0.0,0},  {0, 0, 0}},                  --DS
    }





  end



--Write to SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(12*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*13;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]

    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    
    step_queue_vector[offset+7] = step_queue[i][5]    

    step_queue_vector[offset+8] = step_queue[i][6][1]
    step_queue_vector[offset+9] = step_queue[i][6][2]
    step_queue_vector[offset+10] = step_queue[i][6][3]

    step_queue_vector[offset+11] = step_queue[i][7][1]
    step_queue_vector[offset+12] = step_queue[i][7][2]
    step_queue_vector[offset+13] = step_queue[i][7][3]
  end
  mcm.set_step_footholds(step_queue_vector)
  mcm.set_step_nfootholds(#step_queue)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  calculate_footsteps()
  motion_ch:send'preview'  
end

function state.update()
  --print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if mcm.get_walk_ismoving()==0 then
    return 'done'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
