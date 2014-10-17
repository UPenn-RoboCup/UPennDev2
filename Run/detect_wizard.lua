#!/usr/bin/env luajit
-- process 

dofile'../include.lua'
local ffi = require'ffi'
local torch = require'torch'
local T = require'libTransform'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']

local vector = require'vector'
require'vcm'

-- Debugging flag
local DEBUG = Config.debug.detect

-- Subscribe to important messages
local mesh_ch = si.new_subscriber'mesh0'

-- Some global variables
local n_scanlines, n_returns, scan_angles
local body_pitch_offset
local lidar_z = 0.1  -- TODO

--TODO: robot specific
local body_height = 1.03
if IS_WEBOTS then
  body_pitch_offset = 0
elseif HOSTNAME=='alvin' then
    body_pitch_offset = -2.5/180*pi
else -- teddy
    body_pitch_offset = 0
end


local xyz_global

local function transform(points, data)    
  local rfov = vector.new(data.rfov)*RAD_TO_DEG
  local v_angles0, v_angles = torch.range(rfov[1], rfov[2], 0.25)  
  if v_angles0:size(1) > n_returns then
    v_angles = torch.mul(v_angles0:sub(1, n_returns), DEG_TO_RAD):type('torch.FloatTensor')
  else
    v_angles = torch.mul(v_angles0, DEG_TO_RAD):type('torch.FloatTensor')
  end
    
  --TODO: maybe just build a 3D tensor?
  xyz_global = torch.FloatTensor(n_scanlines, n_returns, 3)
  
  -- Transfrom to x,y,z
  local roll, pitch, pose
  local pre_pose -- this is for scanlines that are not updated
  -- Buffers for points
  local xs = torch.FloatTensor(1, n_returns)
  local ys = torch.FloatTensor(1, n_returns)
  local zs = torch.FloatTensor(1, n_returns)
  local xyz_new = torch.FloatTensor(4, n_returns)
  for i=1,n_scanlines do
    local scanline = points:select(1, i)
    
    xs:copy(scanline)
    ys:copy(scanline)
    zs:copy(scanline)
    
    xs:cmul(torch.cos(v_angles)):mul(math.cos(scan_angles[i]))
    ys:cmul(torch.cos(v_angles)):mul(math.sin(scan_angles[i]))
    zs:cmul(torch.sin(v_angles)):mul(-1):add(lidar_z)
    
    -- Transform to GLOBAL frame
    pitch = data.pitch[i] + body_pitch_offset
    roll = data.roll[i]
    if data.pose[i] then 
      pose = data.pose[i] 
      pre_pose = pose
    else 
      pose = pre_pose
    end
    
    if not pose then pose = wcm.get_robot_pose() end
    
    -- A homogeneous transformation
    local rotY = T.rotY(pitch)
    local rotX = T.rotX(roll)
    local rotZ = T.rotX(pose[3])
    local trans = T.trans(pose[1], pose[2], body_height)
    
    local R = torch.mm(trans, torch.mm(rotZ,torch.mm(rotX, rotY))):type('torch.FloatTensor')
        
    -- Transform to global coordinates
    -- Maybe dumb 
    local xyz_local = torch.cat(torch.cat(torch.cat(xs,ys,1),zs,1),
      torch.ones(1,n_returns):type('torch.FloatTensor'),1)
      
    xyz_new:mm(R, xyz_local)
    xyz_global:select(3,1)[i]:copy(xyz_new[1])
    xyz_global:select(3,2)[i]:copy(xyz_new[2])
    xyz_global:select(3,3)[i]:copy(xyz_new[3])
    
    if DEBUG and i==40 then
      -- print( unpack(vector.new(scanline)) )
      -- print( unpack(vector.new(zs)) )
      -- print('POSE:', unpack(pose))
      -- print(string.format('Roll %.2f, Pitch %.2f \n', roll*RAD_TO_DEG, pitch*RAD_TO_DEG))
      print(xyz_new:size())      
      -- print(unpack(vector.new(xyz_new:select(2,3))))
      -- print('AFTER', unpack(vector.new(pz[i])))
      print(unpack(vector.new(xyz_global:select(3,3)[i])))
      -- Above is good
      
    end

  end

  -- visualize
  
  -- Now we somehow have the point clouds, convert to TCCM
  -- TODO: we may add a libTccm or something in Util
  -- GRID
  -- STRUCT OF A CUBOID
  -- C++ ?

  
    
end

local function plane_detect()
  return
end

mesh_ch.callback = function(skt)  
  -- Only use the last one
  local pdata, ranges = unpack(skt:recv_all())
  local data = munpack(pdata)
  -- Useful params
  n_scanlines, n_returns = unpack(data.dims)
  scan_angles = data.a
  
  -- Point cloud container
  local points = torch.FloatTensor(n_scanlines, n_returns):zero()
  ffi.copy(points:data(), ranges)

  if DEBUG then
  --   print(unpack(vector.new(points:select(1, 40))))
    -- print(unpack(scan_angles))
  end
  
  -- If the laser data is not ready
  if #data.pose==0 then
    print('DISCARD EMPTY LASER CONTAINER...')
    return
  end
    
  -- Transform to cartesian space
  transform(points, data)
  
  -- Model building
  plane_detect()
  
end



local TIMEOUT = 1 / 10 * 1e3  --TODO
local poller = si.wait_on_channels{mesh_ch}
local npoll

-- TODO: for TCCM, we shouldn't use the mesh_wizard
-- just add each scanline to the global map

local function update()  
  npoll = poller:poll(TIMEOUT)
end


if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=nil, update=update, exit=nil}
end


--TODO: add signal to kill properly
while trun do
	update()
	if t - t_debug > debug_interval then
    t_debug = t
    print(string.format('DETECT | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end
