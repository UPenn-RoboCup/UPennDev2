require 'torch'
torch.Tensor = torch.FloatTensor

local libLaser = {}
-- Buffers created ONCE for performance
libLaser.nRays  = 1081
libLaser.points = torch.Tensor(4,libLaser.nRays):zero()
libLaser.FOV    = 270; -- This many degrees
libLaser.resd   = 0.25; -- Kinda rounded, but FOV may not be truly 270
libLaser.res    = libLaser.resd/180*math.pi;
libLaser.angles  = torch.range(0,libLaser.FOV,libLaser.resd)
-- Center the range angles, and convert to radians
libLaser.angles = (libLaser.angles - libLaser.FOV/2) * math.pi/180
libLaser.cosines = torch.cos(libLaser.angles);
libLaser.sines   = torch.sin(libLaser.angles);

-- Start with a mask of all ones to allow ALL points
libLaser.mask = torch.Tensor(libLaser.angles:size()):fill(1)

-- TODO: Make sure the helper functions are working properly!
local function rotx(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the X axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[2][2] = ct;
  r[3][3] = ct;
  r[2][3] = -1*st;
  r[3][2] = st;
  return r
end
libLaser.rotx = rotx

local function roty(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Y axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[3][3] = ct;
  r[1][3] = st;
  r[3][1] = -1*st;
  return r
end
libLaser.roty = roty

local function rotz(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Z axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[2][2] = ct;
  r[1][2] = -1*st;
  r[2][1] = st;
  return r
end
libLaser.rotz = rotz

local function trans(v)
  local t = torch.eye(4);
  t[1][4] = v[1] or 0;
  t[2][4] = v[2] or 0;
  t[3][4] = v[3] or 0;
  return t;
end
libLaser.trans = trans

local function ranges2xyz(ranges,pitch,roll,yaw)
  pitch = pitch or 0;
  roll = roll or 0;
  yaw = yaw or 0;
  local nranges = (#ranges)[1]
  if nranges~=libLaser.nRays then
    print("BAD RANGE INPUT",nranges,libLaser.nRays)
    return;
  end
  -- easier access to data
  local X = libLaser.points;

  -- Put lidar readings into relative cartesian coordinate
  X:resize(4,nranges)
  local xs = X:select(1,1);
  local ys = X:select(1,2);
  xs:copy(ranges):cmul( libLaser.cosines )
  ys:copy(ranges):cmul( libLaser.sines )

  -- Accept only ranges that are sufficiently far away
  -- TODO: Make fast masking!
  local good_cnt = 0;
  for i=1,nranges do
    if ranges[i]>0.25 and libLaser.mask[i]==1 then
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
  X:select(1,3):fill(0); -- z
  X:select(1,4):fill(1); -- extra

  -- Apply the transformation given current roll and pitch
  T = torch.mm(
  libLaser.roty(pitch),libLaser.rotx(roll)
  );
  --T = torch.eye(4);
  -- TODO: Verify that Y:mm(T,X) is not needed
  X:mm(T,X);  --reverse the order because of transpose
  xs = X:select(1,1);
  ys = X:select(1,2);
  local zs = X:select(1,3);

  -- Reset the views
  X:resize(4,nranges)
  xs = X:select(1,1);
  ys = X:select(1,2);
  zs = X:select(1,3);

  -- Return the data
  --print("Contiguous?",xs:isContiguous(),ys:isContiguous(),xs:isContiguous())
  -- NOTE: Can also access via libLaser.points
  return xs,ys,zs;
end
libLaser.ranges2xyz = ranges2xyz

return libLaser