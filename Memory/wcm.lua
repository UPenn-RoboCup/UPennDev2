--------------------------------
-- World Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local memory = require'memory'

-- shared properties
local shared = {}
local shsize = {}

shared.robot = {}
-- Webot pose
shared.robot.pose_gps = vector.zeros(3)
shared.robot.time = vector.zeros(1)
-- 0: unknown, 1: attack, 2: defend
shared.robot.role = vector.ones(1)
shared.robot.pose_vision = vector.zeros(3)

--Combined pose (either odom only / slam only / combined)
shared.robot.pose = vector.zeros(3)

--Odometry 
shared.robot.utorso0 = vector.zeros(3)
shared.robot.utorso1 = vector.zeros(3)

--Odometry pose
shared.robot.odometry = vector.zeros(3)
-- 0: only odom
-- 1: only slam
-- 2: slam + odom
shared.robot.odom_mode = vector.zeros(1)

shared.robot.reset_pose = vector.zeros(1)






-- SLAM pose
shared.slam = {}
shared.slam.pose = vector.zeros(3)


-- Particles for pose estimation
shared.particle = {}
shared.particle.x = vector.zeros(Config.world.nParticle)
shared.particle.y = vector.zeros(Config.world.nParticle)
shared.particle.a = vector.zeros(Config.world.nParticle)
shared.particle.w = vector.zeros(Config.world.nParticle)


-- Auto detected objects
shared.ballfilter = {}
shared.ballfilter.r = vector.zeros(1)
shared.ballfilter.a = vector.zeros(1)
shared.ballfilter.rvar = vector.zeros(1)
shared.ballfilter.avar = vector.zeros(1)


shared.ball = {};
shared.ball.x = vector.zeros(1);
shared.ball.y = vector.zeros(1);
shared.ball.t = vector.zeros(1);
shared.ball.velx = vector.zeros(1);
shared.ball.vely = vector.zeros(1);
shared.ball.dodge = vector.zeros(1);
shared.ball.locked_on = vector.zeros(1);
shared.ball.p = vector.zeros(1);


shared.goal = {}
shared.goal.t = vector.zeros(1)
shared.goal.attack_angle = vector.zeros(1)
shared.goal.defend_angle = vector.zeros(1)

shared.obstacle = {}
shared.obstacle.enable = vector.zeros(1)
shared.obstacle.reset = vector.ones(1)
shared.obstacle.detect = vector.zeros(1)
shared.obstacle.count = vector.zeros(1)
shared.obstacle.v1 = vector.zeros(2)
shared.obstacle.v2 = vector.zeros(2)
shared.obstacle.v3 = vector.zeros(2)

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
