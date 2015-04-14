assert(Config, 'Need a pre-existing Config table!')

local world = {}

world.odomScale = {1,1,1} -- For now IMU not in use
world.use_imu_yaw = true
world.use_gps_pose = false

-- "Ball"
world.ballDiameter = 0.22


-- Associate with the table
Config.world = world

return Config
