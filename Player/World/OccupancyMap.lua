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
  occ = OccMap.retrieve();
	ocm.set_occ_robot_pos(occ.robot_pos);
end

function update()
	-- Vision Update

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
  print("Odometry change: ",uOdometry[1],uOdometry[2],uOdometry[3]);
	OccMap.odometry_update(uOdometry[1], uOdometry[2], uOdometry[3]);
	
	-- shm Update
	occ = OccMap.retrieve();
	ocm.set_occ_map(occ.map);		
end
