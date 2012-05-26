module(..., package.seeall);

require('Config');	
require('Body')
require('shm');
require('vcm');
require('unix'); -- Get Time
require('wcm');
require('mcm');
require('ocm');
require('OccMap');
require('vector');

uOdometry0 = vector.new({0, 0, 0});

odomScale = Config.world.odomScale;

function entry()
  OccMap.init(Config.occ.mapsize, Config.occ.robot_pos[1], 
              Config.occ.robot_pos[2], unix.time());
  nCol = vcm.get_freespace_nCol();
  OccMap.vision_init(nCol);

  occmap = OccMap.retrieve_map();
  ocm.set_occ_map(occmap); 
  occdata = OccMap.retrieve_data();
	ocm.set_occ_robot_pos(occdata.robot_pos);
end 

function odom_update()
	-- Odometry Update
  uOdometry, uOdometry0 = mcm.get_odometry(uOdometry0);

  uOdometry[1] = odomScale[1]*uOdometry[1];
  uOdometry[2] = odomScale[2]*uOdometry[2];
  uOdometry[3] = odomScale[3]*uOdometry[3];

  --Gyro integration based IMU
  if imuYaw==1 then
    yaw = Body.get_sensor_imuAngle(3);
    uOdometry[3] = yaw-yaw0;
    yaw0 = yaw;
--    print("Body yaw:",yaw*180/math.pi, " Pose yaw ",pose.a*180/math.pi)
  end
--  print("Odometry change: ",uOdometry[1],uOdometry[2],uOdometry[3]);
	OccMap.odometry_update(uOdometry[1], uOdometry[2], uOdometry[3]);
end

function vision_update()
  vbound = vcm.get_freespace_vboundB();
  tbound = vcm.get_freespace_tboundB();

  nCol = vcm.get_freespace_nCol();
  OccMap.vision_update(vbound, tbound, nCol, unix.time());
--  print("scanned freespace width "..vbound);
end

function update()
  -- Time decay
  local time = unix.time();
  OccMap.time_decay(time);

	-- Vision Update
  vision_update();

	-- Odometry Update
  odom_update();
	
	-- shm Update
	occmap = OccMap.retrieve_map();
	ocm.set_occ_map(occmap);		
end
