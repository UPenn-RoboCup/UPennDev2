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
local jpeg = require'jpeg'
local unloggers = {}

---------------------------------
-- Shared Memory
require'jcm'
require'hcm'
require'vcm'
local head_camera, lwrist_camera, joint_positions
---------------------------------

---------------------------------
-- Logging and Replaying set up
local loggers = {}
local log_dir = 'Log/'
local open_logfile = function(dev_name)
  -- Set up log file
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local filename = string.format('%s/%s_%s.log',log_dir,dev_name,filetime)
  return io.open(filename,'r')
end

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
    -- read the metadata
    local meta = unloggers.head_camera.meta_unpacker:unpack()
    if not meta then return end
    -- print the metadata
    util.ptable(meta)
    -- read the c_img
    local c_img = unloggers.head_camera.raw_file:read(meta.sz)
    -- Save the c_img, for now... should just replay on the channel...
    print('raw:', #c_img)
    local jimg = jpeg.uncompress(c_img)
    print('jpeg',jimg:width(),jimg:height())
    print(util.color('\n=========\n','green'))
    return meta
  end,
  lwrist_camera = function()
    -- read the metadata
    local meta = unloggers.lwrist_camera.meta_unpacker:unpack()
    if not meta then return end
    -- print the metadata
    util.ptable(meta)
    -- read the c_img
    local c_img = unloggers.lwrist_camera.raw_file:read(meta.sz)
    -- Save the c_img, for now... should just replay on the channel...
    print('lraw:', #c_img)
    local jimg = jpeg.uncompress(c_img)
    print('ljpeg',jimg:width(),jimg:height())
    print(util.color('\n=========\n','green'))
    return meta
  end
}
local log = {}
local log_prefix = 'Log/'
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
local fname = log_prefix..filename
      local f = io.open(fname,'r')
      -- read the whole chunk into memory and then close the descriptor
      local str = f:read('*all')
      f:close()
--local res, off = mp.unpack(str)
--print('norm',res,off)
      local up = mp.unpacker(str)
--local stuff = up:unpack()
--print('stuff',stuff,fname)

      tbl.meta_unpacker = up
--print('unpacker',tbl.meta_unpacker,#str)
--os.exit()
      local raw_file = filename:gsub('meta','raw')
      -- Do not read the raw file a priori
      tbl.raw_file = io.open(log_prefix..raw_file,'r')
      tbl.update = updater
      unloggers[dev]=tbl
    end
	end

end

function log.update()
  local done = true
  for name,l in pairs(unloggers) do
    local has_data = l.update()
    print('has_data',has_data)
    if has_data then done = false end
  end
  if done then shutdown() end
end

function log.exit()
	shutdown()
end

-- Main loop
print'Beginning to log...'
log.entry()
while true do log.update() end
log.exit()
