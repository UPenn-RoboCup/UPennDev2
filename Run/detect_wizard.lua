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


-- Subscribe to important messages
local mesh_ch = si.new_subscriber'mesh0'

-- Some global variables
local n_scanlines, n_returns


local function transform(points, rfov)
  print(rfov[1], rfov[2])
  local planes = torch.DoubleTensor(n_scanlines, 3)
  
  -- 1D tensor of the scan angles
  local v_angles0, v_angles = torch.range(rfov[1], rfov[2], 0.25)
  if v_angles0:size(1) > n_returns then
    v_angles = torch.mul(v_angles0:sub(1, n_returns), DEG_TO_RAD)
  else
    v_angles = torch.mul(v_angles0, DEG_TO_RAD)
  end
  
  -- print(v_angles[1], v_angles[v_angles:size(1)])
  
  -- Transfrom to x,y,z
  for i=1,n_scanlines do
    -- TODO
  end
    
end



mesh_ch.callback = function(skt)
  print('HEY !')
  
  -- Only use the last one
  local pdata, ranges = unpack(skt:recv_all())
  local data = munpack(pdata)
  -- Point cloud container
  n_scanlines, n_returns = unpack(data.dims)
  local points = torch.FloatTensor(n_scanlines, n_returns):zero()
  ffi.copy(points:data(), ranges)
  
  print('n_scanlines:', n_scanlines, 'n_returns:',n_returns)

  
  -- Transform to cartesian space
  transform(points, vector.new(data.rfov)*RAD_TO_DEG)
  
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
