dofile'fiddle.lua'
local K = Body.Kinematics
local T = require'libTransform'
local getch = require'getch'
local P = require'libPlan'
local planner = P.new_planner(K,Body.servo.min_rad,Body.servo.max_rad)

local use_stack = true
local dir = 1
local pathStack, pathIter

local qGoal
local trGoal0 =
	T.trans(0.020,0,.45)
local trGoal1 =
	T.rotZ(120 * DEG_TO_RAD) *
	T.trans(.3,0,-.45) *
	trGoal0
	* T.rotY(120 * DEG_TO_RAD)
	* T.rotZ(-45 * DEG_TO_RAD)

local function process_keycode(keycode,t_diff)
  local char = string.char(keycode)
  local char_lower = string.lower(char)

	if char_lower=='p' then
		local qArm = Body.get_command_position()
		local trGoal
		if dir==1 then trGoal = trGoal1 else trGoal = trGoal0 end
		dir = dir * -1
		if use_stack==true then
			print("IK STACK")
			pathStack, qGoal = planner:line_stack(qArm,trGoal,nil,nil,false)
		else
			print("IK ITERATOR")
			pathIter, qGoal  = planner:line_iter(qArm,trGoal,nil,nil,false)
		end
		is_following = true
	elseif char_lower=='q' then
		local qArm = Body.get_command_position()
		local trGoal
		if dir==1 then trGoal = trGoal1 else trGoal = trGoal0 end
		dir = dir * -1
		qGoal = K.inverse_arm(trGoal,qArm)
		if use_stack==true then
			print("Q STACK")
			pathStack = planner:joint_stack(qArm,qGoal)
		else
			print("Q ITERATOR")
			pathIter = planner:joint_iter(qGoal)
		end
		is_following = true
	elseif char_lower=='w' then
		local qArm = Body.get_command_position()
		local trGoal
		if dir==1 then trGoal = trGoal1 else trGoal = trGoal0 end
		dir = dir * -1
		local posGoal = vector.new{trGoal[1][4],trGoal[2][4],trGoal[3][4]}
		if use_stack==true then
			print("POS STACK")
			pathStack, qGoal = planner:line_stack(qArm,posGoal,nil,nil,false)
		else
			print("POS ITERATOR")
			pathIter, qGoal = planner:line_iter(qArm,posGoal,nil,nil,false)
		end
		is_following = true
	elseif char_lower == 'k' then
		-- Cancel the plan
		is_following = false
	elseif char_lower=='o' then
		use_stack = not use_stack
		if use_stack==true then
			print("STACK")
		else
			print("ITERATOR")
		end
	elseif char_lower==' ' then
		print('PAUSE',is_following)
		is_following = not is_following
	elseif char_lower == '0' then
		Body.set_command_position(vector.zeros(Body.nJoint))
	end

end

-- Start processing
io.flush()
local t0 = unix.time()
t_keycode = 0
while true do
  -- Grab the keyboard character
  local keycode = getch.nonblock()
  -- Measure the timing
  local t = unix.time()
  local t_diff = t - t0
  t0 = t
  -- Process the character
	if keycode and t-t_keycode>.1 then
		t_keycode = t
  	process_keycode(keycode,t_diff)
	end
	-- Update the planning follower
	if is_following==true then
		local qWaypoint
		if use_stack==true then
			qWaypoint = table.remove(pathStack)
		else
			qWaypoint = pathIter(Body.get_command_position())
		end
		if not qWaypoint then
			print('DONE')
			is_following=false
			Body.set_command_position(qGoal)
		else
			Body.set_command_position(qWaypoint)
		end
		--print(vector.new(qWaypoint))
	end
	-- check if following
	-- Sleep a little
	unix.usleep(1e5)
end
