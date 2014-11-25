#!/usr/bin/env luajit
-- process 

dofile'../include.lua'
local ffi = require'ffi'
local torch = require'torch'
local T = require'libTransform'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local unix = require'unix'
local vector = require'vector'
require'vcm'

-- Debugging flag
local DEBUG = Config.debug.detect

-- Subscribe to important messages
local mesh_ch = si.new_subscriber'mesh0'
local kinect_depth_ch = si.new_subscriber'kinect2_depth'

-- Some global variables
local n_scanlines, n_returns, scan_angles
local body_pitch_offset
local lidar_z = 0.1  -- TODO
local max_range = 3 --meters

--TODO: robot specific
local body_height = 1.03
if IS_WEBOTS then
  body_pitch_offset = 0
elseif HOSTNAME=='alvin' then
  body_pitch_offset = -2.5/180*pi
else -- teddy
  body_pitch_offset = 0
end


-- Octomap
local octomap = require'octomap'
-- TODO: read params from shm
octomap.set_resolution(0.01)
octomap.set_range(0.05, 2)


local xyz_global, xyz_local
local roll, pitch, pose

local function transform(points, data)    
  local rfov = vector.new(data.rfov)*RAD_TO_DEG
  local v_angles0, v_angles = torch.range(rfov[1], rfov[2], 0.25)  
  if v_angles0:size(1) > n_returns then
    v_angles = torch.mul(v_angles0:sub(1, n_returns), DEG_TO_RAD):type('torch.FloatTensor')
  else
    v_angles = torch.mul(v_angles0, DEG_TO_RAD):type('torch.FloatTensor')
  end
    
  xyz_global = torch.FloatTensor(n_scanlines*n_returns, 3):zero()
        
  -- Transfrom to x,y,z
  local pre_pose -- this is for scanlines that are not updated
  -- Buffers for points
  local xs = torch.FloatTensor(1, n_returns)
  local ys = torch.FloatTensor(1, n_returns)
  local zs = torch.FloatTensor(1, n_returns)
  for i=1,n_scanlines do
    local scanline = points:select(1, i)
    
    xs:copy(scanline)
    ys:copy(scanline)
    zs:copy(scanline)
    
    -- If the lidar is paning 
    if vcm.get_mesh_direction()==0 then
      -- print('LIDAR IS PANNING...')
      xs:cmul(torch.cos(v_angles)):mul(math.cos(scan_angles[i]))
      ys:cmul(torch.cos(v_angles)):mul(math.sin(scan_angles[i]))
      zs:cmul(torch.sin(v_angles)):mul(-1):add(lidar_z)
    else    
      -- print('LIDAR IS TILTING...')
      -- If the lidar is tilting
      -- TODO: verify if this is correct
      xs:cmul(torch.cos(v_angles)):mul(math.cos(scan_angles[i]))
      ys:cmul(torch.sin(v_angles)):mul(-1)
      zs:cmul(torch.cos(v_angles)):mul(math.sin(scan_angles[i])):mul(-1):add(lidar_z)   
    end    
    
    -- Transform to GLOBAL frame
    pitch = data.pitch[i] + body_pitch_offset
    roll = data.roll[i]
    if data.pose[i] then 
      pose = data.pose[i] 
      pre_pose = pose
    else 
      print('WARNING!!!!!!!!!!!!!!!!!!!')
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
    xyz_local = torch.cat(torch.cat(torch.cat(xs,ys,1),zs,1),
      torch.ones(1,n_returns):type('torch.FloatTensor'),1)
      
    xyz_local = torch.mm(R, xyz_local)   -- 4*360
    xyz_global[{{(i-1)*n_returns+1, i*n_returns},{}}]:copy(xyz_local:sub(1,3):t())
        
  end
  
  if DEBUG then
    -- print(unpack(vector.new(xyz_global[{{10*n_returns+1, 11*n_returns},3}])))
  end

  -- visualize: octovis or matlab
  
    
end

local function plane_detect()
  return
end

local td -- for benchmarking
local saved = false

-- Callback function for mech channel
mesh_ch.callback = function(skt) 
  print('I AM HERE...')
  
   
  -- Only use the last one
  local pdata, ranges = unpack(skt:recv_all())
  local data = munpack(pdata)
  -- Useful params
  n_scanlines, n_returns = unpack(data.dims)
  scan_angles = data.a
  
  -- Point cloud container
  local points = torch.FloatTensor(n_scanlines, n_returns):zero()
  ffi.copy(points:data(), ranges)

  
  -- If the laser data is not ready
  if #data.pose==0 then
    print('DISCARD EMPTY LASER CONTAINER...')
    return
  end
    
  -- Transform to cartesian space
  -- TODO: transform in octomap *maybe* faster
  td = unix.time()
  transform(points, data)
  if DEBUG then
    -- print('Transform the entir scan..', unix.time()-td)
  end
  
  ---[[ Update sensor pose
  octomap.set_origin(torch.DoubleTensor({pose[1], pose[2], body_height}))

  -- Insert point cloud to Octree  
  td = unix.time()
  octomap.add_scan(xyz_global)
  if DEBUG then
    print('Added one full scan.. ', unix.time()-td, '\n')
  end
  --]]
  
  -- min x/y/z, max x/y/z
  octomap.get_horizontal(0.1, -0.5, 1, 0.8, 0.5, 1.5)
  
  -- if unix.time()-td>5 and not saved then
  --   octomap.save_tree('test.bt')
  --   saved = true
  -- end
  --
end


local depths_t, w, h, focal_len
local function update_kinect_depth(data, depths)
  print('Updating kinect depth...')
  -- Useful params
  w, h = unpack(data.dims)
  --TODO: no need to calculate every time
  -- Focal length
  -- f = w/2/tan(fov/2)  fov = 1.2217
  focal_len = w/2/math.tan(1.2217/2)
  
  --TODO: use octomap methods to directly manipulate depths
	local float_depths = ffi.cast('float*', depths)
  local byte_sz = w * h * ffi.sizeof'float'
  -- print('w=', w, 'h=', h, 'FLOAT DEPTH SIZE', byte_sz)
  depths_t = torch.FloatTensor(h, w):zero()
  ffi.copy(depths_t:data(), float_depths, byte_sz)
  
  local rpy, pose, angle = data.rpy, data.pose, data.angle
  
  print('POSE:', pose[1], pose[2], lidar_z)
  print('RPY:', unpack(rpy))
  print('angle:', angle)
  
  
  -- TODO: this takes most of the time
  octomap.add_depth(depths_t, focal_len,
    pose[1], pose[2], lidar_z,
    unpack(rpy))
end



-- Callback function for kinect depth channel
kinect_depth_ch.callback = function(skt)
  -- Only use the last one
  local pdata, depths = unpack(skt:recv_all())
  local data = munpack(pdata)
  update_kinect_depth(data, depths)
end



local TIMEOUT = 1 / 10 * 1e3  --TODO
local poller = si.wait_on_channels{mesh_ch, kinect_depth_ch}
local npoll


local t_entry = unix.time()
local function update()  
  npoll = poller:poll(TIMEOUT)
  if hcm.get_tree_save()==1 then
    octomap.save_tree('test.bt')
    hcm.set_tree_save(0)
  end
  
end


if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=nil, update=update, exit=nil, 
    update_kinect_depth=update_kinect_depth, }
end


--TODO: add signal to kill properly

while trun do
	update()
	if t - t_debug > debug_interval then
    t_debug = t
    print(string.format('DETECT | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end
