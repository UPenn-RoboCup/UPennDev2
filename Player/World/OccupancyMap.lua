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
if (string.find(Config.platform.name, 'Webots'))  then
  yawScale = 1.38;
else
  yawScale = 1;
end
lastTime = 0;


function entry()
  lastTime = unix.time();
  OccMap.init(Config.occ.mapsize, Config.occ.robot_pos[1], 
              Config.occ.robot_pos[2], lastTime);
  nCol = vcm.get_freespace_nCol();
--  OccMap.vision_init(nCol);

  occmap = OccMap.retrieve_map();
  ocm.set_occ_map(occmap); 
  occdata = OccMap.retrieve_data();
	ocm.set_occ_robot_pos(occdata.robot_pos);
end 

function cur_odometry()
  if mcm.get_walk_isFallDown() == 1 then
    print('FallDown and Reset Occupancy Map')
    OccMap.reset();
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
  --  print('Body yaw change', uOdometry[3]);
  end
--print("Odometry change: ",uOdometry[1],uOdometry[2],uOdometry[3]);
  return uOdometry;
end

function odom_update()
  uOdometry = cur_odometry();
  --print("Odometry change: ",uOdometry[1],uOdometry[2],uOdometry[3]);
	OccMap.odometry_update(uOdometry[1], uOdometry[2], uOdometry[3]);
end

function vision_update()
  vbound = vcm.get_freespace_vboundB();
  tbound = vcm.get_freespace_tboundB();

  nCol = vcm.get_freespace_nCol();
  OccMap.vision_update(vbound, tbound, nCol, unix.time());
--  print("scanned freespace width "..nCol);
end

lastPos = vector.zeros(3); 
function velocity_update()
  curTime = unix.time();
  uOdonmetry = cur_odometry();
--  print(curTime - lastTime);
  vel = (uOdometry - lastPos); -- / (curTime - lastTime);
  ocm.set_occ_vel(vel);

--  print(vel[1], vel[2], vel[3]);
  lastPos = uOdometry; 
  lastTime = curTime;
end

function obs_in_occ()
  print('try find obstacle in occmap'); 
 -- start = unix.time();
  obstacle = OccMap.get_obstacle();
  for i = 1 , #obstacle do
    print('centroid')
    util.ptable(obstacle[i].centroid);
    print('angle_range')
    print(obstacle[i].angle_range[1] * 180 / math.pi, 
          obstacle[i].angle_range[2] * 180 / math.pi);
    print('nearest')
    util.ptable(obstacle[i].nearest);
  end
--  endd = unix.time();
--  print(endd - start);
end

function update()
  
--  velocity_update();


  -- Time decay
  local time = unix.time();
  OccMap.time_decay(time);

	-- Vision Update
  vision_update();

	-- Odometry Update
  odom_update();
	
	-- shm Update
  odom = OccMap.retrieve_odometry();
  ocm.set_occ_odom(vector.new({odom.x, odom.y, odom.a}));
  --print('odom from map',odom.x..' '..odom.y..' '..odom.a);
	occmap = OccMap.retrieve_map();
	ocm.set_occ_map(occmap);		

  local reset = ocm.get_occ_reset();
  if reset == 1 then
    OccMap.reset();
    print('reset occmap in OccupancyMap');
    ocm.set_occ_reset(0);
  end

  local get_obstacle = ocm.get_occ_get_obstacle();
  if get_obstacle == 1 then
    obs_in_occ();
    print("get obstacles from occmap");
    ocm.set_occ_get_obstacle(0);
  end

end

function exit()
end
