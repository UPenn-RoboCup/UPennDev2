local state = {}
state._NAME = ...

local Body = require'Body'
local K    = Body.Kinematics
local util = require'util'
local vector = require'vector'
require'hcm'

local t_entry, t_update

local x_offset_from_handle = -0.30

local function update_approach()
    -- These are the relative coordinates of the handle
  local handle = vector.pose( hcm.get_door_handle() )
  
  --[[
  vector.cross(
    {math.cos(handle_pose.a),math.sin(handle_pose.a),0})
  --]]
  -- Find the desired pose to approach
  local offset = vector.pose()
  -- Stay just a bit in front of the door
  offset.x = x_offset_from_handle
  -- Open with the left hand if on the left side
  offset.y = -util.sign(handle.y) * K.shoulderOffsetY

  print('handle',handle)
  print('offset',offset)

  local relative_waypoint = util.pose_global(offset, handle)

  -- Convert local door coordinates to
  -- global waypoints
  local pose = vector.pose(wcm.get_robot_pose())
  local global_door_approach = util.pose_global(relative_waypoint, pose)
  
  print('pose',pose)
  print('relative wp', relative_waypoint )
  print('global wp', global_door_approach)

  -- Set the one waypoint for now
  hcm.set_motion_waypoints(global_door_approach)
  -- Only one waypoint for now.
  -- TODO: Maybe advanced approached with astar?
  hcm.set_motion_nwaypoints(1)
  -- global waypoint
  hcm.set_motion_waypoint_frame(1)
end

function state.entry()
  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  update_approach()

end

function state.update()
  --print(state._NAME..' Update' )

  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state