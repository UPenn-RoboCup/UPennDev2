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
  print'Shutting down the Cameras...'
  for _,logger in ipairs(loggers) do
    logger.file:close()
    print('Closed log',logger.name)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local devs = {
  head_camera = function()
    -- read the metadata
    local data = head_camera.meta_unpacker:unpack()
    util.ptable(data)
    -- read the c_img
  end
}
local logs = {}
local unloggers = {}
function log.entry()
	
	-- Set up listeners based on the input arguments
	for _,filename in ipairs(arg) do
    -- Only take meta file names
    local e = filename:find('_%d')
    local dev = filename:sub(1,e)
    local updater = devs[dev]
    if type(updater)=='function' then
      local tbl = {}
      tbl.name = dev
      local f = io.open(filename,'r')
      -- read the whole chunk into memory and then close the descriptor
      local str = f:read('*all')
      tbl.meta_unpacker = mp.unpacker(str)
      f:close()
      local raw_file = filename:gsub('meta','raw')
      -- Do not read the raw file a priori
      tbl.raw_file = io.open(raw_file,'r')
      tbl.update = updater
      table.insert(unloggers,tbl)
    end
	end

end

function log.update()
  for _,l in pairs(unloggers) do
    unloggers.update()
  end
end

function log.exit()
	shutdown()
end

-- Main loop
print'Beginning to log...'
log.entry()
while true do log.update() end
log.exit()