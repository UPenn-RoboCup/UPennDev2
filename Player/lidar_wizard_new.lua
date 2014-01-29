-----------------------------------------------------------------
-- Hokuyo LIDAR Wizard
-- Reads lidar scans and saves to shared memory
-- (c) Stephen McGill, 2013
---------------------------------
dofile'include.lua'

-- Libraries
local Body       = require'Body'
local signal     = require'signal'
local carray     = require'carray'
local mp         = require'msgpack'
local util       = require'util'
local simple_ipc = require'simple_ipc'
local libHokuyo  = require'libHokuyo'
local DEG_TO_RAD = Body.DEG_TO_RAD
local RAD_TO_DEG = Body.RAD_TO_DEG

-- Setup the Hokuyos array
local hokuyos = {}

-- Initialize the Hokuyos
local hokuyo = libHokuyo.new_hokuyo('/dev/ttyACM0')
hokuyo.name = 'Front'
local lidar_ch = simple_ipc.new_publisher'lidar'
hokuyo.callback = function(data)
	--local scan = carray.float(data,768)
	-- Mid right left
	--print("DATA",#data,#scan,scan[385],scan[129],scan[641])
	local meta = {}
	meta.t     = Body.get_time()
	meta.n     = 769 --768
	meta.fov   = 270 --239.77
	meta.res   = 360 / 1024
	local ret  = lidar_ch:send{mp.pack(meta),data}
end

-- Ensure that we shutdown the devices properly
local function shutdown()
  print'Shutting down the Hokuyos...'
  for i,h in ipairs(hokuyos) do
    h:stream_off()
    h:close()
    print('Closed Hokuyo',i)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Begin to service
os.execute('clear')
assert(#hokuyos>0,"No hokuyos detected!")
print( util.color('Servicing '..#hokuyos..' Hokuyos','green') )

local main = function()
  local main_cnt = 0
  local t0 = Body.get_time()
  while true do
    main_cnt = main_cnt + 1
    local t_now = Body.get_time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      debug_str = util.color(debug_str,'yellow')
      for i,h in ipairs(hokuyos) do
        debug_str = debug_str..string.format(
        '\n\t%s Hokuyo:\t%5.1fHz\t%4.1f ms ago',h.name, 1/h.t_diff, (t_now-h.t_last)*1000)
      end
      os.execute('clear')
      print(debug_str)
      t0 = t_now
      main_cnt = 0
    end
    coroutine.yield()
  end
end

table.insert(hokuyos,hokuyo)
libHokuyo.service( hokuyos, main )
