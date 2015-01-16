-- libVision
-- (c) 2014 Stephen McGill
-- General Detection methods

--SJ: dummy libvision (for non-robocup use)

local libVision = {}
--[[
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
local T = require'libTransform'
local mp = require'msgpack.MessagePack'
local vector = require'vector'
local util = require'util'
local zlib = require'zlib.ffi'
local si = require'simple_ipc'
require'wcm'
require'hcm'
require'gcm'
--]]



function libVision.get_metadata()
end

function libVision.send()
end

-- Set the variables based on the config file
function libVision.entry(cfg, body)
end

function libVision.update(img)
  t=unix.time()
  t_last = t
end

return libVision
