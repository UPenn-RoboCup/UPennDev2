#!/usr/bin/env luajit
-- process 

dofile'../include.lua'
local ffi = require'ffi'
local torch = require'torch'
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
local lidar_z = 0.1  -- TODO

local function transform(points, data)    
  local rfov = vector.new(data.rfov)*RAD_TO_DEG
  local v_angles0, v_angles = torch.range(rfov[1], rfov[2], 0.25)  
  if v_angles0:size(1) > n_returns then
    v_angles = torch.mul(v_angles0:sub(1, n_returns), DEG_TO_RAD):type('torch.FloatTensor')
  else
    v_angles = torch.mul(v_angles0, DEG_TO_RAD):type('torch.FloatTensor')
  end
  
  -- print(v_angles[1], v_angles[v_angles:size(1)])
  
  local px = torch.FloatTensor(n_scanlines, n_returns)
  local py = torch.FloatTensor(n_scanlines, n_returns)
  local pz = torch.FloatTensor(n_scanlines, n_returns)
  -- local px = torch.DoubleTensor(n_scanlines, n_returns)
  -- local py = torch.DoubleTensor(n_scanlines, n_returns)
  -- local pz = torch.DoubleTensor(n_scanlines, n_returns)
  
  -- Transfrom to x,y,z
  for i=1,n_scanlines do
    local scanline = points:select(1, i)
    local xs = px:select(1,i):copy(scanline)
    local ys = py:select(1,i):copy(scanline)
    local zs = pz:select(1,i):copy(scanline)
    
    xs:cmul(torch.cos(v_angles)):mul(math.cos(scan_angles[i]))
    ys:cmul(torch.cos(v_angles)):mul(math.sin(scan_angles[i]))
    zs:cmul(torch.sin(v_angles)):mul(-1):add(lidar_z)
    
    if DEBUG and i==40 then 
      -- print( unpack(vector.new(scanline)) )
      -- print( unpack(vector.new(zs)) )
      -- return 
    end

  end
  
  -- visualize
  
  -- Now we somehow have the point clouds, convert to TCCM
    
end



mesh_ch.callback = function(skt)
  print('HEY !')
  
  -- Only use the last one
  local pdata, ranges = unpack(skt:recv_all())
  local data = munpack(pdata)
  -- Useful params
  n_scanlines, n_returns = unpack(data.dims)
  scan_angles = data.a
  -- Point cloud container
  local points = torch.FloatTensor(n_scanlines, n_returns):zero()
  ffi.copy(points:data(), ranges)

  -- if DEBUG then
  --   print(unpack(vector.new(points:select(1, 40))))
  --   return
  -- end
  
  -- Transform to cartesian space
  transform(points, data)
  
end



local TIMEOUT = 1 / 10 * 1e3  --TODO
local poller = si.wait_on_channels{mesh_ch}
local npoll

local function update()
  
  -- print('DETECT WIZARD')
  
  npoll = poller:poll(TIMEOUT)
    
  -- if npoll==0 then print('NO MESH RECEIVED') end
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
    print(string.format('World | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end
