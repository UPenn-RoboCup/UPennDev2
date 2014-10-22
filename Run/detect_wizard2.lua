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

-- For transformation
local body_pitch_offset
local lidar_z = 0.1  -- TODO
--Robot specific
local body_height = 1.03
if IS_WEBOTS then
  body_pitch_offset = 0
elseif HOSTNAME=='alvin' then
    body_pitch_offset = -2.5/180*pi
else -- teddy
    body_pitch_offset = 0
end

-- Some global variables
local n_scanlines, n_returns, scan_angles
local ranges_fov, min_view, max_view
local xyz

-- Map
local tccm = require'tccm'
local MAP = {}
MAP.res = 0.02
-- xmin, ymin, xmax, ymax
MAP.bounds = {-1, 0.1, 1, 2}
tccm.set_resolution(MAP.res)
tccm.set_boundaries(unpack(MAP.bounds))

local function transform(points, data)    
  -- Robot pose info
  local roll, pitch, yaw = unpack(data.rpy)
  local pose = data.pose
	local lidar_angle = data.angle -- yaw for chest lidar (for now)
  
  -- Laser angles
  local rfov = ranges_fov*RAD_TO_DEG
  local v_angles0, v_angles = torch.range(rfov[1], rfov[2], 0.25)  
  if v_angles0:size(1) > n_returns then
    v_angles = torch.mul(v_angles0:sub(1, n_returns), DEG_TO_RAD):type('torch.FloatTensor')
  else
    v_angles = torch.mul(v_angles0, DEG_TO_RAD):type('torch.FloatTensor')
  end
  
  -- Transfrom to cartesian and store in buffers
  local xs = torch.FloatTensor(1,n_returns):copy(points)
  local ys = torch.FloatTensor(1,n_returns):copy(points)
  local zs = torch.FloatTensor(1,n_returns):copy(points)
  
  xs:cmul(torch.cos(v_angles)):mul(math.cos(lidar_angle))
  ys:cmul(torch.cos(v_angles)):mul(math.sin(lidar_angle))
  zs:cmul(torch.sin(v_angles)):mul(-1):add(lidar_z)
  
  -- A homogeneous transformation
  local rotY = T.rotY(pitch)
  local rotX = T.rotX(roll)
  local rotZ = T.rotX(pose[3])
  local trans = T.trans(pose[1], pose[2], body_height)
  
  local R = torch.mm(trans, torch.mm(rotZ,torch.mm(rotX, rotY))):type('torch.FloatTensor')
      
  -- Transform to global coordinates
  -- Maybe dumb 
  xyz = torch.cat(torch.cat(torch.cat(xs,ys,1),zs,1),
    torch.ones(1,n_returns):type('torch.FloatTensor'),1)
    
  xyz = torch.mm(R, xyz)  -- now in global coordinates
  
  if DEBUG and math.abs(lidar_angle)<1*DEG_TO_RAD then
    -- print( unpack(vector.new(zs)) )
    -- print(unpack(vector.new(xyz[3])) )
    -- Above is good
  end

end


local function update_map()
  tccm.grow_map(xyz:sub(1,3))
end

local function plane_detect()
  return
end 



-- Update the part map
local function update(data, ranges)
  -- Params
	local ranges_fov0 = vcm.get_mesh_fov()
	-- Check if updated parameters
	if not ranges_fov or ranges_fov~=ranges_fov0 then
		ranges_fov = ranges_fov0
  	min_view, max_view = unpack(ranges_fov)
		print('FOV updated..')
	end
  -- Check fov
  local n, res = data.n, data.res
  local fov = n * res
	assert(fov > max_view-min_view, 'Not enough FOV available')
  
	-- Find the offset for copying lidar readings into the mesh
  local fov_offset = min_view / res + n / 2
	-- Round the offset (0 based offset)
  local offset_idx = math.floor(fov_offset + 0.5)
  
  -- Round to get the number of returns for this scanline
  n_returns = math.floor((max_view - min_view) / res + 0.5)
  print("n_returns", n_returns, max_view, min_view, res)
	local dest = torch.FloatTensor(1, n_returns)
  
  -- Copy the ranges
	local float_ranges = ffi.cast('float*', ranges)
	local byte_sz = n_returns * ffi.sizeof'float'
	ffi.copy(dest:data(), float_ranges + offset_idx, byte_sz)
  
  -- Transform to cartesian space
  transform(dest, data)

  -- Build cuboid map
  update_map()
  
  -- Model building
  plane_detect()
  
end


-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end

local lidar_ch = si.new_subscriber'lidar0'
function lidar_ch.callback(skt)
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, ranges)
end

si.wait_on_channels({lidar_ch}):start()


