require 'torch'
--require 'ffi/torchffi'
torch.Tensor = torch.FloatTensor

local Config = {}

local LIDAR0 = {}
-- Static configurations
LIDAR0.resd    = 0.25;
LIDAR0.res     = LIDAR0.resd/180*math.pi;
LIDAR0.nRays   = 1081;
LIDAR0.angles  = torch.range(0,(LIDAR0.nRays-1)*LIDAR0.resd,LIDAR0.resd)
LIDAR0.angles = (LIDAR0.angles - 135) * math.pi/180
LIDAR0.cosines = torch.cos(LIDAR0.angles);
LIDAR0.sines   = torch.sin(LIDAR0.angles);
LIDAR0.offsetx = 0;--0.137;
LIDAR0.offsety = 0;
LIDAR0.offsetz = 0;--0.54;  --from the body origin (not floor)
LIDAR0.mask    = torch.Tensor(LIDAR0.angles:size()):fill(1);
-- Be very conservative to ensure no interpolated antenna obstacles
--LIDAR0.mask(1:190) = 0;
--LIDAR0.mask(end-189:end) = 0;
LIDAR0.present = 1;
LIDAR0.startTime = 0;

-- Dynamic Configurations (per process)
-- TODO: should this even be defined?
LIDAR0.ranges = torch.Tensor(LIDAR0.nRays);
LIDAR0.timestamp = 0;
Config.LIDAR0 = LIDAR0;

local IMU = {}
-- Static configurations
-- Dynamic Configurations (per process)
IMU.roll = 0;
IMU.pitch = 0;
IMU.yaw = 0;
Config.IMU = IMU;

return Config;