---------------------------------
-- Logging manager for Team THOR
-- (c) Stephen McGill 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Body = require'Body'
require 'unix'
local simple_ipc = require'simple_ipc'
local util = require'util'
local mp = require'msgpack'
local unloggers = {}

---------------------------------
-- Shared Memory
require'jcm'
require'hcm'
require'vcm'
local head_camera, lwrist_camera, joint_positions
---------------------------------

-- Close the camera properly upon Ctrl-C
local signal = require 'signal'
local function shutdown()
  print'Shutting down the unloggers...'
  for name,unlogger in pairs(unloggers) do
    unlogger.raw_file:close()
    print('Closed log',name)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local devs = {
  head_camera = function()
    local tbl = unloggers.head_camera
    local meta = tbl.meta_unpacker:unpack()
    if not meta then return end
    local c_img = tbl.raw_file:read(meta.sz)
    -- send along the channel
    tbl.pub:send{mp.pack(meta),c_img}
    return meta
  end,
  lwrist_camera = function()
    local tbl = unloggers.lwrist_camera
    local meta = tbl.meta_unpacker:unpack()
    if not meta then return end
    local c_img = tbl.raw_file:read(meta.sz)
    -- send along the channel
    tbl.pub:send{mp.pack(meta),c_img}
    return meta
  end
}
local log = {}
function log.entry()
	
	-- Set up listeners based on the input arguments
	for _,filename in ipairs(arg) do
    -- Only take meta file names
    local e = filename:find('_%d')
    local dev = filename:sub(1,e-1)
    local updater = devs[dev]
    if type(updater)=='function' then
      local tbl = {}
      tbl.name = dev
      local fname = LOG_DIR..filename
      local f = io.open(fname,'r')
      -- read the whole chunk into memory and then close the descriptor
      local str = f:read('*all')
      f:close()
      local up = mp.unpacker(str)
      tbl.meta_unpacker = up
      local raw_file = filename:gsub('meta','raw')
      -- Do not read the raw file a priori
      tbl.raw_file = io.open(LOG_DIR..raw_file,'r')
      tbl.update = updater
      tbl.pub = simple_ipc.new_publisher(dev)
      unloggers[dev]=tbl
    end
	end

end

function log.update()
  local done = true
  for name,l in pairs(unloggers) do
    local meta = l.update()
    if meta then
      done = false
      util.ptable(meta)
      print(util.color('=====','yellow'))
    end
  end
  if done then shutdown() end
  print()
end

function log.exit()
	shutdown()
end

-- Main loop
print'Unlogging...'
log.entry()
while true do log.update() end
log.exit()
