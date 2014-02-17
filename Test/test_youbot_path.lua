dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'libTransform'
local getch = require'getch'
local P = require'libPlan'
local planner = P.new_planner(K)

local use_stack = true
local dir = 1

local diff = vector.new{.1,.1,-.1}
local zrot = 0
local yrot = 0

local diff = vector.new{.3,.1,-.5}
--local diff = vector.new{.1,.1,-.1}
local zrot = -45 * DEG_TO_RAD
local yrot = 120 * DEG_TO_RAD

local function process_keycode(keycode,t_diff)
  local char = string.char(keycode)
  local char_lower = string.lower(char)

  if char==' ' then
    -- Debugging
    local grip = jcm.get_gripper_command_position()
    local qArm = Body.get_command_position()
    local fk = K.forward_arm(qArm)
    local cur_vel = mcm.get_walk_vel()
    print('cur_vel',cur_vel)
    print()
    print('grip',grip[1])
    print()
    print('qArm',qArm*RAD_TO_DEG)
    print()
    print(T.tostring(fk))
    return
  end

	if char_lower=='p' then
		local qArm = Body.get_command_position()
		local fk = K.forward_arm(qArm)
		local trGoal
		if dir==1 then
			trGoal = T.trans(unpack(diff))
			* fk
			* T.rotY(yrot)
			* T.rotZ(zrot)
		else
			trGoal = T.trans(0.020,0,.45)
		end
		dir = dir * -1
		local pathStack, pathIter
		if use_stack==true then
			print("STACK")
			pathStack = planner:line_stack(qArm,trGoal)
		else
			print("ITERATOR")
			pathIter  = planner:line_iter(qArm,trGoal)
		end
		while true do
			local qWaypoint
			if use_stack==true then
				qWaypoint = table.remove(pathStack)
			else
				qWaypoint = pathIter(Body.get_command_position())
			end
			if not qWaypoint then break end
			--print(vector.new(qWaypoint))
			Body.set_command_position(qWaypoint)
			--unix.usleep(1e6)
		end
		print'DONE'
	elseif char_lower=='o' then
		use_stack = not use_stack
		if use_stack==true then
			print("STACK")
		else
			print("ITERATOR")
		end
	end

end

-- Start processing
io.flush()
local t0 = unix.time()
while true do
  -- Grab the keyboard character
  local keycode = getch.block()
  -- Measure the timing
  local t = unix.time()
  local t_diff = t - t0
  t0 = t
  -- Process the character
  process_keycode(keycode,t_diff)
end
