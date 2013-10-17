dofile('../include.lua')

require'unix'

------------------
-- Libraries
local simple_ipc = require'simple_ipc'
local mp = require'msgpack'
local Body = require'Body'
------------------

------------------
-- Shared memory
require'jcm'
------------------

-- Load log files
local logfile = io.open(arg[1], 'r')
local log_str = logfile:read('*a')
logfile:close()
local data_unpacker = mp.unpacker(log_str)

-- Log data broadcasting
local head_lidar_ch, chest_lidar_ch
local mesh_udp_ch, cam_udp_ch
head_lidar_ch  = simple_ipc.new_publisher'head_lidar'
chest_lidar_ch  = simple_ipc.new_publisher'chest_lidar'
mesh_udp_ch = udp.new_sender(
  Config.net.operator.wired, Config.net.mesh )
cam_udp_ch = udp.new_sender(
  Config.net.operator.wired, Config.net.head_camera )


-- Sleep time
local tsleep = 1/40 * 1e6

local meta_tbl, has_more = data_unpacker:unpack()
while meta_tbl do
  -- Detect the channel to use
  if type(meta_tbl) == 'table' then
    print('which channel', meta_tbl.name)
    if meta_tbl.name == 'headlidar' then
      --print('AM HERE')
      head_lidar_ch:send( mp.pack(meta_tbl) )
    elseif meta_tbl.name == 'chestlidar' then
      chest_lidar_ch:send( mp.pack(meta_tbl) )
    elseif meta_tbl.name == 'chest_lidar' or
            meta_tbl.name == 'head_lidar' then
      -- Deal with c_mesh
      c_mesh = data_unpacker:unpack()
      mesh_udp_ch:send( mp.pack(meta_tbl)..c_mesh )
	    unix.usleep(tsleep)
    elseif meta_tbl.name == 'hcam' then
      c_color = data_unpacker:unpack()
      cam_udp_ch:send( mp.pack(meta_tbl)..c_color )
      unix.usleep(tsleep)
    elseif meta_tbl.name == 'imu' then
      jcm.set_sensor_rpy( meta_tbl.rpy )
      jcm.set_sensor_gyro( meta_tbl.gyro )
    elseif meta_tbl.name == 'sensor' then
      jcm.set_sensor_position( meta_tbl.pos )
      jcm.set_sensor_velocity( meta_tbl.vel )
    elseif meta_tbl.name == 'actuator' then
      jcm.set_actuator_command_position( meta_tbl.pos )
      jcm.set_actuator_command_velocity( meta_tbl.vel )
    else
      print('Unknown logging data')
    end

    meta_tbl, has_more = data_unpacker:unpack()
  else
  	-- Just for debugging
  	print(meta_tbl)
  end

	-- sleep
	unix.usleep(tsleep)
end
