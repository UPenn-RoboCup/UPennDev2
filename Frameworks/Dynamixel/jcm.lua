---------------------------------------------------------------------------
-- Joint Communication Module
---------------------------------------------------------------------------
require('vector')
require('shm_util')

local nJointArm = 6;

jcm = {}

local shared_data = {}
-- Left arm
-- Observations
shared_data.left_arm_position = vector.zeros(nJointArm) 
shared_data.left_arm_velocity = vector.zeros(nJointArm)
-- Commands
shared_data.left_arm_command_position = vector.zeros(nJointArm)
shared_data.left_arm_command_velocity = vector.zeros(nJointArm)
-- Right arm
-- Observations
shared_data.right_arm_position = vector.zeros(nJointArm) 
shared_data.right_arm_velocity = vector.zeros(nJointArm)
-- Commands
shared_data.right_arm_command_position = vector.zeros(nJointArm)
shared_data.right_arm_command_velocity = vector.zeros(nJointArm)

---------------------------------
-- Perform the SHM initialization
---------------------------------
shm_util.init_shm_module(jcm, 'jcm', shared_data)

--------------------------------
-- Populate the Joint IDs Lookup
--------------------------------
jcm.right_nx_ids = {}
jcm.right_mx_ids = {}
jcm.right_all_ids = {}
jcm.left_nx_ids = {}
jcm.left_mx_ids = {}
jcm.left_all_ids = {}

for i=1,18,2 do
	if i<12 then
		table.insert(jcm.right_nx_ids,i)
		table.insert(jcm.left_nx_ids,i+1)
	else
		table.insert(jcm.right_mx_ids,i)
		table.insert(jcm.left_mx_ids,i+1)
	end
	table.insert(jcm.right_all_ids,i)
	table.insert(jcm.left_all_ids,i+1)
end

return jcm