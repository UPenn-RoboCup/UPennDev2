local	NITE_JOINT_HEAD,
NITE_JOINT_NECK,

NITE_JOINT_LEFT_SHOULDER,
NITE_JOINT_RIGHT_SHOULDER,
NITE_JOINT_LEFT_ELBOW,
NITE_JOINT_RIGHT_ELBOW,
NITE_JOINT_LEFT_HAND,
NITE_JOINT_RIGHT_HAND,

NITE_JOINT_TORSO,

NITE_JOINT_LEFT_HIP,
NITE_JOINT_RIGHT_HIP,
NITE_JOINT_LEFT_KNEE,
NITE_JOINT_RIGHT_KNEE,
NITE_JOINT_LEFT_FOOT,
NITE_JOINT_RIGHT_FOOT = 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15

local test_joint = NITE_JOINT_RIGHT_HAND

local skeleton = require 'skeleton'
-- Get the number of users being tracked
local n_users = skeleton.open()
print( "Number of Skeletons:", n_users )

--for n=1,10 do
while true do
	local visible = skeleton.update()
	if visible and type(visible)=="table" and #visible==n_users then
		for u,v in pairs(visible) do
			if v then
				local pos,orient,ts = skeleton.joint(u,test_joint);
				if pos and orient and ts then
					print('==Position==')
					for j,stats in pairs(pos)do
						if type(stats)=="table" then
							print( j, unpack(stats) )
						else
							print( j, stats )
						end
					end
					print('==Orientation==')
					for j,stats in pairs(orient)do
						if type(stats)=="table" then
							print( j, unpack(stats) )
						else
							print( j, stats )
						end
					end
					print('==Timestamp==')
					print('ts',ts)
					print()
				else
					print("Bad joint!")
				end--joint
			else
				--print("Invisible user",u)
			end--v
		end--uv pairs
	else
		print("Bad user tracking!",rc)
	end
end
-- Shutdown the skeleton
skeleton.shutdown()
