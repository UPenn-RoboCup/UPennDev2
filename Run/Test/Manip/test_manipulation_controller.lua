dofile('../../include.lua')

require('mcm')
require('dcm')
require('Config')
t = require('Transform')

joint = Config.joint

anipulation_state = {}

Manipulation_state.__index = Manipulation_state
Manipulation_state.__mtstring = 'Manipulation_state'
setmetatable(Manipulation_state, Motion_state)

manipulation_controller = require('manipulation_controller')
--manipulation_controller.THOR_MC_initialize()

function set_real_T_array_from_table(real_T_array, array_offset, number_array)
	for count=1, #number_array 
		do manipulation_controller.real_T_setitem(real_T_array,array_offset-1+count,number_array[count])
	end
end

function real_T_array_to_table(real_T_array, array_first, array_last)
	v = {}
	for index=array_first, array_last
		do 
			local val = manipulation_controller.real_T_getitem(real_T_array,index)
			n = index-array_first+1
			v[n] = val
			print(index..' '..val..' '..n..' '..v[n])
				
	end
	return v
end


inputs = manipulation_controller.THOR_MC_U
outputs = manipulation_controller.THOR_MC_Y
step =0

function Manipulation_state:update()

while true do
step=step+1
print(step)
--val = 0.01
inputs.TimeStep = 0.01

print(inputs.TimeStep)

inputs.JointSpaceGoalEnabled = true

print("JSG enabled "..inputs.JointSpaceGoalEnabled)

set_real_T_array_from_table(inputs.JointSpaceVelocityGoal,0,{0.1,0.2,0.3,0.4,0.5,0.6})

manipulation_controller.THOR_MC_step()

if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	print("faulted")
end

t= real_T_array_to_table(outputs.JointVelocityCmds,0,13)
end
--mcm:set_desired_r_hand_pose(t,
print("end")

manipulation_controller.THOR_MC_terminate()
end

Manipulation_state.update()
