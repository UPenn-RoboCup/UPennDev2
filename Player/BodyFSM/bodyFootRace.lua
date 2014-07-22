local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local libStep = require'libStep'
-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')

local robocupplanner = require'robocupplanner'


-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

require'mcm'
require'gcm'


local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg
local last_ph = 0
local kickAngle
local count

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  motion_ch:send'hybridwalk'
  last_ph = 0

  phase = 0
end

local function update_velocity()
  local vStep

--  local targetX = 6.0
  local targetX = 4.5


  if phase==0 then
    target_pose = {targetX,0,0}
  elseif phase==1 then --rotate
    target_pose = {targetX,0,math.pi}
  elseif phase==2 then --return
    target_pose = {0,0,math.pi}
  end


  local pose = wcm.get_robot_pose()
  vStep,reached = robocupplanner.getVelocity(pose,target_pose,0)  
  mcm.set_walk_vel(vStep)  
  if reached then phase = phase+1 end
  return reached
end


local count 

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  local reached=false
  -- Save this at the last update time
  t_update = t
  local check_ph = 0.95

  local ph = mcm.get_status_ph()
  if last_ph<check_ph and ph>=check_ph then     
    reached=update_velocity() 
  end
  last_ph = ph
  if phase==3 then
    mcm.set_walk_stoprequest(1) 
    return "done"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
