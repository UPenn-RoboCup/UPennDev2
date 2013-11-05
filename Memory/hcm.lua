--------------------------------
-- Human Communication Module --
-- (c) 2013 Stephen McGill    --
--------------------------------
local vector = require'vector'
local memory = require'memory'



local shared_data = {}
local shared_data_sz = {}

--SJ: 

-- Desired joint properties
shared_data.joints = {}
-- x,y,z,roll,pitch,yaw
shared_data.joints.plarm  = vector.zeros( 6 )
shared_data.joints.prarm  = vector.zeros( 6 )
-- TODO: 6->7 arm joint angles
shared_data.joints.qlarm  = vector.zeros( 7 )
shared_data.joints.qrarm  = vector.zeros( 7 )
-- 3 finger joint angles
shared_data.joints.qlgrip = vector.zeros( 3 )
shared_data.joints.qrgrip = vector.zeros( 3 )

shared_data.joints.qlshoulderyaw = vector.zeros( 1 )
shared_data.joints.qrshoulderyaw = vector.zeros( 1 )
-- Teleop mode
-- 1: joint, 2: IK
shared_data.joints.teleop = vector.ones( 1 )





-- Motion directives
shared_data.motion = {}
shared_data.motion.velocity = vector.zeros(3)
-- Emergency stop of motion
shared_data.motion.estop = vector.zeros(1)


--Head look angle
shared_data.motion.headangle = vector.zeros(2)


-- Waypoints
-- {[x y a][x y a][x y a][x y a]...}
shared_data.motion.waypoints  = vector.zeros(3*maxWaypoints)
-- How many of the waypoints are actually used
shared_data.motion.nwaypoints = vector.zeros(1)
-- Local or global waypoint frame of reference
-- 0: local
-- 1: global
shared_data.motion.waypoint_frame = vector.zeros(1)


-------------------------------
-- Task specific information --
-------------------------------
-- Wheel/Valve
shared_data.wheel = {}
-- This has all values: the right way, since one rpc call
-- {handlepos(3) handleyaw handlepitch handleradius}
shared_data.wheel.model = vector.new({0.36,0.00,0.02, 0, 0*Body.DEG_TO_RAD,0.20})
-- Target angle of wheel
shared_data.wheel.turnangle = vector.zeros(1)


-- Door Opening
shared_data.door = {}
--door_hand, hinge_xyz(3), door_r, grip_offset_x
shared_data.door.model = vector.new({
	1, --door_hand, 1 for left, 0 for right
	0.45,0.85,-0.15, --Hinge XYZ pos from robot frame
	-0.60, --Door radius, negative - left hinge, positive - right hinge
	-0.05, --The X offset of the door handle (from door surface)
	})
shared_data.door.yaw = vector.zeros(1) --The current angle of the door
shared_data.door.yaw_target = vector.new({-20*math.pi/180}) --The target angle of the door 



-- Drill gripping
shared_data.tool={}
-- posxyz(3), yawangle
shared_data.tool.model = vector.new({0.45,0.15,-0.05,  0*Body.DEG_TO_RAD})




-- Dipoles for arbitrary grabbing
-- TODO: Use this in place of the wheel/door?
shared_data.left = {}
shared_data.left.cathode = vector.zeros(3)
shared_data.left.anode = vector.zeros(3)
-- strata (girth) / angle of attack / climb (a->c percentage)
shared_data.left.grip = vector.zeros(3)
----
shared_data.right = {}
shared_data.right.cathode = vector.zeros(3)
shared_data.right.anode = vector.zeros(3)
-- strata (girth) / angle of attack / climb (a->c percentage)
shared_data.right.grip = vector.zeros(3)

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
