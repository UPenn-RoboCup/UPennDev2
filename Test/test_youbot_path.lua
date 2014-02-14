dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'libTransform'
local getch = require'getch'
local P = require'libPlan'
local planner = P.new_planner(K)

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
		local trGoal = T.trans(.05,.05,-.05) * fk
		local pathStack = planner:line(qArm,trGoal)
		--[[
		for _,qWaypoint in ipairs(pathStack) do
			print(vector.new(qWaypoint))
			Body.set_command_position(qWaypoint)
			unix.usleep(1e4)
		end
		--]]
		while true do
			local qWaypoint = table.remove(pathStack)
			if not qWaypoint then break end
			print(vector.new(qWaypoint))
			Body.set_command_position(qWaypoint)
			unix.usleep(1e5)
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
