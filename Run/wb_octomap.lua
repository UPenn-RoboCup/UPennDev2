dofile('include.lua')

-- Libraries
local simple_ipc = require 'simple_ipc'
local msgpack = require 'msgpack'
local carray = require 'carray'
local Octomap = require'Octomap'
local torch = require'torch'
torch.Tensor = torch.FloatTensor

-- Global vars
require 'unix'
require 'Params'
local actuator_positions = {};
local actuator_commands = {}
actuator_commands = carray.double(#Params.jointNames);
for i = 1,#Params.jointNames do
  actuator_commands[i] = 0;
  actuator_positions[i] = 0;
end

-- Prune function
local LIDAR0 = {}
-- Static configurations
LIDAR0.resd    = 0.25;
LIDAR0.res     = LIDAR0.resd/180*math.pi;
LIDAR0.nRays   = 1081;
LIDAR0.angles  = torch.range(0,(LIDAR0.nRays-1)*LIDAR0.resd,LIDAR0.resd)
LIDAR0.angles = (LIDAR0.angles - 135) * math.pi/180
LIDAR0.cosines = torch.cos(LIDAR0.angles);
LIDAR0.sines   = torch.sin(LIDAR0.angles);
LIDAR0.offsetx = 0;
LIDAR0.offsety = 0;
LIDAR0.offsetz = 0;--from the body origin (not floor)
LIDAR0.mask    = torch.Tensor(LIDAR0.angles:size()):fill(1);
-- Take in a CArray
local X = torch.Tensor(4,LIDAR0.nRays):zero()
local Y = torch.Tensor(4,LIDAR0.nRays):zero()
local function ranges2xyz(ranges,pitch,roll)
  pitch = pitch or 0;
  roll = roll or 0;
  local nranges = (#ranges)[1]
  if nranges~=LIDAR0.nRays then
    print("BAD RANGE INPUT")
    return;
  end

  -- Put lidar readings into relative cartesian coordinate
  X:resize(4,nranges)
  local xs = X:select(1,1);
  local ys = X:select(1,2);
  xs:copy(ranges):cmul( LIDAR0.cosines )
  ys:copy(ranges):cmul( LIDAR0.sines )

  -- Accept only ranges that are sufficiently far away
  -- TODO: Make fast masking!
  local good_cnt = 0;
  for i=1,nranges do
    if ranges[i]>0.25 and LIDAR0.mask==1 then
      good_cnt = good_cnt+1;
      xs[good_cnt] = xs[i];
      ys[good_cnt] = ys[i];
    end
  end
  -- Resize to include just the good readings
  nranges = good_cnt;
  if nranges==0 then
    print('No good readings after initial checks.')
    return
  end

  -- Reset the view
  X:resize(4,nranges)
  Y:resize(4,nranges)
  X:select(1,3):fill(0); -- z
  X:select(1,4):fill(1); -- extra

  -- Apply the transformation given current roll and pitch
  -- TODO: check if transpose...
  -- TODO: this is unstable!
  T = torch.mm(
  libSlam.roty(pitch),libSlam.rotx(roll)
  );
  --T = torch.eye(4);
  Y:mm(T,X);  --reverse the order because of transpose
  xs = Y:select(1,1);
  ys = Y:select(1,2);
  local zs = Y:select(1,3);

  -- Reset the views
  X:resize(4,nranges)
  Y:resize(4,nranges)
  xs = Y:select(1,1);
  ys = Y:select(1,2);
  zs = Y:select(1,3);

  -- Return the data
  print("Contiguous?",xs:isContiguous(),ys:isContiguous(),xs:isContiguous())
  return xs,ys,zs;
end

-- IPC channels
local actuator_pub_channel = simple_ipc.new_publisher('actuator_cmd')

local camera_channel = simple_ipc.new_subscriber('camera')
camera_channel.callback = function()
  local res = camera_channel:receive()
  --  print('camera chanel ', #res)
end

local imu_channel = simple_ipc.new_subscriber('imu')
imu_channel.callback = function()
  local res = imu_channel:receive()
  local imu_tbl = msgpack.unpack(res)
  --print('IMU:', unpack(imu_tbl) )
end

local actuator_channel = simple_ipc.new_subscriber('actuator')
actuator_channel.callback = function()
  local res = actuator_channel:receive()
  actuator_positions = msgpack.unpack(res)
end

local lidar_channel = simple_ipc.new_subscriber('lidar')
lidar_channel.callback = function()
  local ts, has_more = lidar_channel:receive();
  if not has_more then
    print("Bad lidar ts!")
    return
  end
  local ranges, has_more = lidar_channel:receive();
  local lidar_ts = tonumber(ts);
  local lidar_ranges = carray.float( ranges );
  local ranges_s = torch.FloatStorage( 1081, lidar_ranges:pointer() )
  local ranges_t = torch.FloatTensor( ranges_s );
  print( lidar_ranges:pointer() )
  print( lidar_ranges[500], ranges_t[1], ranges_s[1] );
  --print( ranges_s, ranges_t );
  --print( #lidar_ranges, #ranges_s, (#ranges_t)[1], #(#ranges_t) )
  ranges2xyz(ranges_t,pitch,roll)

  --print(lidar_ts," Lidar: ", #lidar_ranges)
  -- Change the lidar head to scan
  local pitch = 10*math.cos( ts ) + 20
  actuator_commands[2] = pitch*math.pi/180
end

local wait_channels = {imu_channel, camera_channel, actuator_channel, lidar_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels )

--local channel_timeout = 30
local channel_timeout = -1
local t0 = unix.time()
while true do
  local n_poll = channel_poll:poll(channel_timeout)
  -- Send actuator commands after each update?
  local ret = actuator_pub_channel:send( tostring(actuator_commands) )
  local t = unix.time()
  local fps = 1/(t-t0)
  t0 = t;
  local debug_msg = string.format(
  "Updating at %.3f FPS",
  fps
  )
  print( debug_msg )
end
