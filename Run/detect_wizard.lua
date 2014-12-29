#!/usr/bin/env luajit
-- process 

dofile'../include.lua'
local Body = require(Config.dev.body)
local ffi = require'ffi'
local torch = require'torch'
local T = require'libTransform'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local unix = require'unix'
local vector = require'vector'
require'vcm'
require'hcm'

-- Debugging flag
local DEBUG = Config.debug.detect
local t0 = Body.get_time()
local t, t_debug, debug_interval = t0, t0, 1

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
  body_pitch_offset = -2.5/180*math.pi
else -- teddy
  body_pitch_offset = 0
end


-- Octomap
local octomap = require'octomap'
-- TODO: read params from shm
octomap.set_resolution(0.01)
octomap.set_range(0.05, 2)
-- Set the map parameters
octomap.set_occupancyThres(0.7)
octomap.set_prob_hit_miss(0.8, 0.3)


local xyz_global, xyz_local
local roll, pitch, pose

local function transform_mesh(points, data)    
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
    roll = data.roll[i] + math.pi/2 -- TODO: a hack...
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

end

local function plane_detect()
  return
end

local td -- for benchmarking
local saved = false

-- Callback function for mech channel
mesh_ch.callback = function(skt)
  print('MESH_CH CALL BACK...')
  --if hcm.get_octomap_update()==0 then return end
   
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
  td = unix.time()
  transform_mesh(points, data)
  if DEBUG then
    -- print('Transform the entir scan..', unix.time()-td)
  end
  
  octomap.set_origin(torch.DoubleTensor({pose[1], pose[2], body_height}))

  -- Insert point cloud to Octree  
  td = unix.time()
  octomap.add_scan(xyz_global)
  
  -- min x/y/z, max x/y/z
  -- octomap.get_horizontal(0.1, -0.5, 1, 0.8, 0.5, 1.5)
  
end



-- Update the Head transform
local trKinect, vKinect, trNeck, trNeck0
local dtrKinect, kinectPitch, kinectRoll, kinectPos
local kinectPos = Config.head.kinectPos or {0,0,0}

local function update_head(md)
	if not Body then return end
	-- Get from Body...
	
  local head = md.head
  local headBias = hcm.get_kinect_bias()
  head[1] = head[1] - headBias[1]  
	
  -- Use imu value to recalculate kinect transform every frame
  local rpy = md.rpy
  trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(rpy[2])
  * T.trans(Config.head.neckX, 0, Config.head.neckZ)
  trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
	
	kinectPitch = headBias[2]
	kinectRoll = headBias[3]
	kinectYaw = headBias[4]

	dtrKinect = T.trans(unpack(kinectPos))
  * T.rotY(kinectPitch or 0)
  * T.rotX(kinectRoll or 0)
  * T.rotZ(kinectYaw or 0)

  trKinect = trNeck * dtrKinect
  -- Grab the pose
  vKinect = vector.new(T.position6D(trKinect)) -- tenor to vetor
end


local depths_t, w, h, focal_len
local function update_kinect_depth(data, depths)
  if hcm.get_octomap_update()==0 then return end 
  -- Useful params
  w, h = data.width, data.height
  --TODO: no need to calculate every time
  -- Focal length
  -- f = w/2/tan(fov/2)  fov = 1.2217
  focal_len = w/2/math.tan(1.2217/2)
  
  --TODO: use octomap methods to directly manipulate depths
	local float_depths = ffi.cast('float*', depths)
  local byte_sz = w * h * ffi.sizeof'float'
  print('w=', w, 'h=', h)
  depths_t = torch.FloatTensor(h, w):zero()
  ffi.copy(depths_t:data(), float_depths, byte_sz)
  
  
  -- update head transform
  update_head(data)
  
  local rpy, pose, angle = data.rpy, data.pose, data.angle
  
  -- Convert to meters
  if not IS_WEBOTS then
    depths_t:mul(1/1000)
  end
    
  octomap.add_depth(depths_t, focal_len, 
    unpack(vKinect) -- sensor_origin w.r.t global reference
  )
    
  -- octomap.get_horizontal(0.1, -0.5, 0.9, 1.5, 0.5, 1.1)  
end


-- Callback function for kinect depth channel
kinect_depth_ch.callback = function(skt)
  print('getting data from depth_ch...')
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
  -- save the current tree
  if hcm.get_octomap_save()==1 then
    octomap.save_tree('../Data/from_log.bt')
    hcm.set_octomap_save(0)
  end
  -- clear the map
  if hcm.get_octomap_clear()==1 then
    octomap.clear_tree()
    hcm.set_octomap_clear(0)
    hcm.set_octomap_update(0)
  end  
  -- segmentation methods
  if hcm.get_octomap_get_door()==1 then
    -- 1: # of plane to detect
    -- 2: max # of iterations
    -- 3: error thres for RANSAC
    -- 4: [opt] 1 means first cluster according to normals
    n_planes, max_iter, eps = 1, 1500, 0.1
    inliers = octomap.get_door(n_planes, max_iter, eps)
    if inliers==0 then
      print("FAIL TO DETECT A DOOR!!!")
    else

      -- SVD for computing the plane
      u, s, v = torch.svd(inliers)
      abcd = v:select(2, 4)
      door_yaw = math.atan2(abcd[2], abcd[1])*RAD_TO_DEG
      if (door_yaw>90) then
        door_yaw = door_yaw - 180
      elseif (door_yaw<-90) then
        door_yaw = 180 + door_yaw
      end
      plane_pos = torch.mean(inliers, 1)
    
      print("normal:", unpack(vector.new(abcd)))
      print("orientation:", door_yaw)
      print("pos:", unpack(vector.new(plane_pos)))
    
      --[[ debugging msg
      print("u size:", unpack(vector.new(u:size())))
      print("s size:", unpack(vector.new(s:size())))
      print("v size:", unpack(vector.new(v:size())))
      check_error = torch.dot(inliers:select(1, 1), abcd)
      print(unpack(vector.new(check_error)))
      --]]
    end
    
    hcm.set_octomap_get_door(0)
  end
end


if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=nil, update=update, exit=nil, 
    update_kinect_depth=update_kinect_depth, }
end


local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

while true do
	update()
	if t - t_debug > debug_interval then
    t_debug = t
    print(string.format('DETECT | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end
