--------------------------------
-- Human Communication Module --
-- (c) 2013 Stephen McGill    --
--------------------------------
local vector = require'vector'
local memory = require'memory'
local maxWaypoints = 10
local DEG_TO_RAD = math.pi/180


local shared_data = {}
local shared_data_sz = {}

--SJ: Now we use this variable to control inner-states 
-- Set 1 to proceed, and -1 to reverse
shared_data.state={}
shared_data.state.proceed = vector.zeros(0)


--This variable is used for target transform based tele-op and fine tuning
shared_data.hands={}

--This variable should contain CURRENT hand transforms
shared_data.hands.left_tr = vector.zeros(6)
shared_data.hands.right_tr = vector.zeros(6)

--This variable should contain TARGET hand transforms
shared_data.hands.left_tr_target = vector.zeros(6)
shared_data.hands.right_tr_target = vector.zeros(6)


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

--Body height Target
shared_data.motion.bodyHeightTarget = vector.zeros(1)


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
shared_data.wheel.model = vector.new({0.36,0.00,0.02, 0, 0*DEG_TO_RAD,0.20})
-- Target angle of wheel
shared_data.wheel.turnangle = vector.zeros(1)




--Small valve (which requires one handed operation)
shared_data.smallvalve = {}
-- This has all values: the right way, since one rpc call
-- {pos(3) roll_start roll_end}
shared_data.smallvalve.model = vector.new({0.50,0.25,0.02,
			 -20*DEG_TO_RAD, 90*DEG_TO_RAD})


--Debris model
shared_data.debris = {}
-- {pos(3) yaw}
shared_data.debris.model = vector.new({0.50,0.25,0.02, 0})









-- Door Opening
shared_data.door = {}
--door_hand, hinge_xyz(3), door_r, grip_offset_x
shared_data.door.model = vector.new({	
	0.45,0.85,-0.15, --Hinge XYZ pos from robot frame
	-0.60, --Door radius, negative - left hinge, positive - right hinge
	-0.05, --The X offset of the door handle (from door surface)
	0.05, --The Y offset of the knob axis (from gripping pos)
	})
shared_data.door.yaw = vector.zeros(1) --The current angle of the door
shared_data.door.yaw_target = vector.new({-20*math.pi/180}) --The target angle of the door 



-- Drill gripping
shared_data.tool={}

--The model of drill,  posxyz(3), yawangle
shared_data.tool.model = vector.new({0.45,0.15,-0.05,  0*DEG_TO_RAD})

-- The positions to start and end cutting
shared_data.tool.cutpos1 = vector.new({0.40,0.17,0, 0})
shared_data.tool.cutpos2 = vector.new({0.40,-0.13,0, 0})



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
