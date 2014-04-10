----------------------------------------
-- Touchscreen Motion processor
-- Listens to touch messages from ZeroMQ
-- (c) Stephen McGill, 2013
----------------------------------------
dofile'include.lua'
-- Libraries
local unix = require'unix'
local util = require'util'
local mp = require'msgpack'
local simple_ipc, poller = require'simple_ipc'
local tou_ch = simple_ipc.new_publisher'touch'
-- Open the log file
local LOG_PREFIX = 'touch'
local LOG_DIR = HOME..'/Logs/'
local LOG_TIMESTAMP = '04.10.2014.12.30.08'
--local LOG_TIMESTAMP = '04.10.2014.15.23.56'
-- TODO: Parse the TIMESTAMP
local LOG_META_FILENAME = LOG_DIR..LOG_PREFIX..'_m_'..LOG_TIMESTAMP..'.log'
local LOG_RAW_FILENAME = LOG_DIR..LOG_PREFIX..'_r_'..LOG_TIMESTAMP..'.log'
print('Opening',LOG_META_FILENAME)
local f_m = io.open(LOG_META_FILENAME,'r')

local ts, t_real = nil, unix.time()
local function process_metadata( meta )
	local timestamp = meta.TIMESTAMP
	-- Remove our added field
	meta.TIMESTAMP = nil
	-- Set the initial timestamp
	if not ts then ts = timestamp end
	-- See how long to sleep
	local t_sleep = timestamp - ts
	ts = timestamp
	-- Sleep a bit to simulate real data
	-- TODO: Make this optional
	unix.usleep(t_sleep*1e6)
	-- Send to the channel
	tou_ch:send(mp.pack(meta))
end

-- Must use an unpacker...
local unpacker = mp.unpacker(2048)
local buf, nbuf = f_m:read(512),0
while buf do
	nbuf = nbuf + #buf
	local res,left = unpacker:feed(buf)
	local tbl = unpacker:pull()
	while tbl do
		-- Process the data
		process_metadata(tbl)
		-- Try to get more data
		tbl = unpacker:pull()
	end
	buf = f_m:read(left)
end
f_m:close()