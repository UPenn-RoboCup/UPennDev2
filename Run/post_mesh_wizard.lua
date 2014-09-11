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



local function transformation()
  local planes = torch.DoubleTensor(n_scanlines, 3)
  
  local v_angles = torch.DoubleTensor()
  
  for i=1,n_scanlines do
    local scanline = mesh:select(1,i)
    local scanline_c = scanline:data()
    local pan_angle = scan_angles[i]
    
    local temp_px

    -- TODO
  end
    
end


mesh_ch.callback = function(skt)
  local meshes = skt:recv_all()
  
  print('HEY !')
  
  -- Only use the last one
	local mesh = mp.unpack(meshes[#meshes])
  
  -- TODO: Projection
  -- transformation(mesh)
  
end



local TIMEOUT = 1 / 10 * 1e3  --TODO
local poller = si.wait_on_channels{mesh_ch}
local npoll

local function update()
  
  -- print('POST MESH WIZARD')
  
  npoll = poller:poll(TIMEOUT)
    
  if npoll==0 then print('NO MESH RECEIVED') end
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
