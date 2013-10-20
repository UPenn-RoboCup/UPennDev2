local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

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

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  calculate_footsteps()
  motion_ch:send'preview'  
end

function calculate_footsteps()
  local step_queue

--For webots with 10ms

  step_queue={
    {{},2,          0.5, 0.5,   {0,0.0,0}},
    {{0.27,0,0},0,  0.5, 2,     {0,-0.0,0}, {0,0.20,0.15}},   --LS
    {{0.27,0,0},1,  1,   2,     {0,0.0,0},  {0,0.20,0.15}},    --RS
    {{},2,          1,   2,     {0,0.0,0}},                  --DS
    {{0.27,0,0},0,  1,   2,     {0,0.0,0}, {0.15,0.20,0.0}},--LS
    {{0.27,0,0},1,  1,   2,     {0,0.0,0}, {0.15,0.20,0.0}},--RS
    {{},2,          0.5, 2,     {0,0.0,0}},                  --DS
  }
  
--For actual robot
--[[
  step_queue={
    {{},2,                  2,  2,    {0,0.0,0}},
    {{0.27,0,0},0,  3, 6, {0,-0.0,0}, {0,0.20,0.15}},   --LS
    {{0.27,0,0},1,  3, 6, {0,0.0,0},  {0,0.20,0.15}},    --RS
    {{},2,          2, 0,  {0,0.0,0}},                  --DS
    {{0.27,0,0},0,  3, 6,   {0,0.0,0}, {0.15,0.20,0.0}},--LS
    {{0.27,0,0},1,  3, 6,   {0,0.0,0}, {0.15,0.20,0.0}},--RS
    {{},2,          2, 4,  {0,0.0,0}},                  --DS
  }
--]]
end


function state.update()
  --print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if mcm.get_walk_ismoving()>0 then
    return 'done'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
