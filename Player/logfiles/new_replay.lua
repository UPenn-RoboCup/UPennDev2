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
require'wcm'
------------------

-- Load log files
local logfile = io.open(arg[1], 'r')
local log_str = logfile:read('*a')
logfile:close()
local data_unpacker = mp.unpacker(log_str)

-- Log data broadcasting
local head_lidar_ch  = simple_ipc.new_publisher'head_lidar'
local chest_lidar_ch  = simple_ipc.new_publisher'chest_lidar'

local mesh_udp_ch = udp.new_sender(
  Config.net.operator.wired, Config.net.mesh )
local cam_udp_ch = udp.new_sender(
  Config.net.operator.wired, Config.net.head_camera )
local fd_udp_ch = udp.new_sender(
  Config.net.operator.wired, Config.net.feedback )


function send_status_feedback()
  local data={}
  data.larmangle = Body.get_larm_command_position()
  data.rarmangle = Body.get_rarm_command_position()
  data.waistangle = Body.get_waist_command_position()
  data.neckangle = Body.get_head_command_position()
  data.llegangle = Body.get_lleg_command_position()
  data.rlegangle = Body.get_rleg_command_position()
  data.lgrip =  Body.get_lgrip_command_position()
  data.rgrip =  Body.get_rgrip_command_position()

  --Pose information
  data.pose =  wcm.get_robot_pose()    
  data.pose_odom =  wcm.get_robot_pose_odom()
  data.pose_slam =  wcm.get_slam_pose()
  data.rpy = Body.get_sensor_rpy()
  data.body_height = mcm.get_camera_bodyHeight()
  data.battery =  0

  local datapacked = mp.pack(data)

  local ret,err = fd_udp_ch:send( datapacked)
end


-- Sleep time
local tsleep = 1/100 * 1e6

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
      local c_mesh = data_unpacker:unpack()
      -- For some reason there are empty table stored
      while type(c_mesh) == 'table' do
      	print('WARNING: got c_mesh as table',unpack(c_mesh))
        c_mesh = data_unpacker:unpack()
      end
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
    elseif meta_tbl.name == 'fsr' then
      jcm.set_sensor_lfoot( meta_tbl.lfoot )
      jcm.set_sensor_rfoot( meta_tbl.rfoot )
    else
      print('Unknown logging data')
    end

    meta_tbl, has_more = data_unpacker:unpack()
  else
  	-- Just for debugging
  	print('Table expected, got', type(meta_tbl))
  end

  -- Send feedback to operator
  send_status_feedback()

	-- sleep
	unix.usleep(tsleep)
end
