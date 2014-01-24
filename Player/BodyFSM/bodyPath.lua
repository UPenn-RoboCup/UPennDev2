local state = {}
state._NAME = ...

local Body = require'Body'
local libMap = require'libMap'

local timeout = 10.0  -- Replan every 10 seconds
local t_entry, t_update, t_exit

-- The map is static, so import it once
local map = libMap.open_map('map.pgm')
-- Make the initial cost to go
local goal = wcm.get_map_goal()
map:new_goal(goal)

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
	
	-- Check the goal, and update if needed
	local cur_goal = wcm.get_map_goal()
	if goal~=cur_goal then
		goal = cur_goal
		map:new_goal(goal)
	end
	
	-- Plan the from our current position
	local pose = wcm.get_robot_pose()
	local path = map:new_path(pose)
	
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  --TODO: Check whether all FSMs have done initialzing 
  return 'done'

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
