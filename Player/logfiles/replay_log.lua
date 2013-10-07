dofile('../include.lua')

require'unix'

local simple_ipc = require'simple_ipc'
local mp = require'msgpack'
local Body = require'Body'

-- Load log files
local logfile = io.open(arg[1], 'r')
local log_str = logfile:read('*a')
logfile:close()
local data_unpacker = mp.unpacker(log_str)

-- Log data broadcasting
local head_lidar_ch
head_lidar_ch  = simple_ipc.new_publisher'head_lidar'

-- Sleep time
local tsleep = 1/40 * 1e6

local meta_tbl = data_unpacker:unpack()
while meta_tbl do
	head_lidar_ch:send( mp.pack(meta_tbl) )
	meta_tbl = data_unpacker:unpack()

  print('Unlogging...', meta_tbl.t)
	-- sleep
	unix.usleep(tsleep)
end
