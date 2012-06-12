module(... or "", package.seeall);

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
require('walk');

uOdometry0 = vector.new({0, 0, 0});

odomScale = Config.world.odomScale or Config.walk.odomScale;
imuYaw = Config.world.imuYaw or 0;
yaw0 = 0;
yawScale = 1.38


function entry()
  OccMap.init(Config.occ.mapsize, Config.occ.robot_pos[1], 
              Config.occ.robot_pos[2], unix.time());
  nCol = vcm.get_freespace_nCol();
--  OccMap.vision_init(nCol);

  occmap = OccMap.retrieve_map();
  ocm.set_occ_map(occmap); 
  occdata = OccMap.retrieve_data();
	ocm.set_occ_robot_pos(occdata.robot_pos);
end 

function odom_update()
  if mcm.get_walk_isFallDown() == 1 then
    print('FallDown and Reset Occupancy Map')
    OccMap.odometry_reset();
  end

	-- Odometry Update
  uOdometry, uOdometry0 = mcm.get_odometry(uOdometry0);

  uOdometry[1] = odomScale[1]*uOdometry[1];
  uOdometry[2] = odomScale[2]*uOdometry[2];
  uOdometry[3] = odomScale[3]*uOdometry[3];

  --Gyro integration based IMU
  if imuYaw==1 then
    yaw = yawScale * Body.get_sensor_imuAngle(3);
    uOdometry[3] = yaw-yaw0;
    yaw0 = yaw;
--    print("Body yaw:",yaw*180/math.pi) --, " Pose yaw ",pose.a*180/math.pi)
  end
--  print("Odometry change: ",uOdometry[1],uOdometry[2],uOdometry[3]);
	OccMap.odometry_update(uOdometry[1], uOdometry[2], uOdometry[3]);
end

function vision_update()
  vbound = vcm.get_freespace_vboundB();
  tbound = vcm.get_freespace_tboundB();

  nCol = vcm.get_freespace_nCol();
  OccMap.vision_update(vbound, tbound, nCol, unix.time());
--  print("scanned freespace width "..nCol);
end

function update()
  
  vel = walk.get_velocity();
--  print('velocity: '..vel[1],vel[2],vel[3]);
  ocm.set_occ_vel(vel);


  -- Time decay
  local time = unix.time();
--  OccMap.time_decay(time);

	-- Vision Update
  vision_update();

	-- Odometry Update
  odom_update();
	
	-- shm Update
  odom = OccMap.retrieve_odometry();
  ocm.set_occ_odom(vector.new({odom.x, odom.y, odom.a}));
--  print(odom.x..' '..odom.y..' '..odom.a);
	occmap = OccMap.retrieve_map();
	ocm.set_occ_map(occmap);		

  local reset = ocm.get_occ_reset();
  if reset == 1 then
    OccMap.reset();
    print('reset occmap in OccupancyMap');
    ocm.set_occ_reset(0);
  end

end

function exit()
end
