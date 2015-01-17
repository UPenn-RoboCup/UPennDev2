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
local motion_ch = simple_ipc.new_publisher('MotionFSM!')

local t_entry, t_update, t_exit, t_stage
local nwaypoints, wp_id
local waypoints = {}

local pose, target_pose
local step_planner
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg


--for deflection test
local sh1,sh2 = 0.10, 0.0
local step1,step2 = 0.25, 0.25

--[[
--This is for webots
step_queues={
   {
    {{step1,0,0},0,  1, 3, 1,   {0,-0.04}, {0,sh1,sh2}   ,  {-step1/2  ,Config.walk.footY+0.04}},   --LS    
   },

   --weight shift to center
    {
    {{0,0,0},   2,    0.1, 2, 0.1,   {0,0},  {0,0,0}},    
    },
  
   {
    {{step1,0,0},1,   1, 3, 1,  {0.0,0},  {0,sh1,sh2}},    --RS    
   }
}


--weight shift to center
    {
  
    },
  

--]]

--local sh1,sh2 = 0.10, 0.05

local sh1,sh2 = 0.10, 0.05
local step1,step2 = 0.0, 0.0

local side_adj = 0.04


--This is for webots
step_queues={
   {
    {{step1,0,0},0,  0.5,2,0.5,   {0,-side_adj}, {0,sh1,sh2}   ,  {-step1/2  ,Config.walk.footY+side_adj}},   --LS    

   },

   {
    {{0,0,0},    2,  0.1, 1, 0.1,   {0,0},  {0,0,0}},    
   },

   {
    {{step1,0,0},1,   0.5,2,0.5,  {0,side_adj},  {0,sh1,sh2},  {-step1/2  ,-Config.walk.footY-side_adj}},    --RS    
   },

   {
    {{0,0,0},    2,  0.1, 1, 0.1,   {0,0},  {0,0,0}},    
   },
}





local stage = 1
local ready_for_input = true

local function calculate_footsteps(stage)
  local step_queue = step_queues[stage]
  --Write to SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(15*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*15;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]

    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    
    step_queue_vector[offset+7] = step_queue[i][5]    

    step_queue_vector[offset+8] = step_queue[i][6][1]
    step_queue_vector[offset+9] = step_queue[i][6][2]
    step_queue_vector[offset+10] = 0

    step_queue_vector[offset+11] = step_queue[i][7][1]
    step_queue_vector[offset+12] = step_queue[i][7][2]
    step_queue_vector[offset+13] = step_queue[i][7][3]

    if step_queue[i][8] then
      step_queue_vector[offset+14] = step_queue[i][8][1]
      step_queue_vector[offset+15] = step_queue[i][8][2]
    else
      step_queue_vector[offset+14] = 0
      step_queue_vector[offset+15] = 0
    end

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

  stage = 1
  calculate_footsteps(stage)
  motion_ch:send'stair'  
  mcm.set_stance_singlesupport(1)
  is_started = true  
  t_stage = t_entry
end


function state.update()
  local t  = Body.get_time()

  --print(state._NAME..' Update' ) 
  -- Get the time of update
  
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if mcm.get_walk_ismoving()==0 and t-t_stage>0.5 then    
    if ready_for_input then print("Ready")
      ready_for_input = false
    end    
    if stage==#step_queues then 
      motion_ch:send'stop'  
      print("ended")
      return 'done'
    elseif hcm.get_state_proceed()==1 then       
      hcm.set_state_proceed(0)
      stage = stage+1
      calculate_footsteps(stage)
      motion_ch:send'stair'  
      ready_for_input = true
      t_stage = t
    end
  else 
    hcm.set_state_proceed(0) --no pogo stick
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  mcm.set_stance_singlesupport(0)
end

return state
