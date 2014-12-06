dofile'../include.lua'
-- local LOG_DATE = '12.04.2014.09.31.54'

-- local LOG_DATE = '12.05.2014.13.34.15'
local LOG_DATE = '12.05.2014.13.28.58'
-- local LOG_DATE = '12.05.2014.14.06.51'
-- local LOG_DATE = '12.05.2014.14.06.43'

local libLog = require'libLog'
local replay_mesh = libLog.open(HOME..'/Data/', LOG_DATE, 'mesh')
local metadata = replay_mesh:unroll_meta()
local logged_mesh = replay_mesh:log_iter()

print('Unlogging', #metadata, 'mesh from', LOG_DATE)

local util = require'util'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local mesh_ch = si.new_publisher'mesh0'


local get_time = unix.time
local metadata_t0 = metadata[1].t
local t0

for i, metadata_mesh, payload_mesh in logged_mesh do
	if i%2==0 then
		print('Count', i)
	end

	local t = get_time()
	t0 = t0 or t
	local dt = t - t0

	local metadata_dt = metadata_mesh.t - metadata_t0
	local t_sleep = metadata_dt-dt
  -- if t_sleep>0 then unix.usleep(1e6*t_sleep) end

  unix.usleep(1e6)
	mesh_ch:send({mp.pack(metadata_mesh), payload_mesh})
      
end
