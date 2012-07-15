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

maxOb = 5;
obs = {};
obs.num = 0;
obs.centroid_x = vector.zeros(maxOb);
obs.centroid_y = vector.zeros(maxOb);
obs.left_range = vector.zeros(maxOb);
obs.right_range = vector.zeros(maxOb);
obs.nearest_x = vector.zeros(maxOb);
obs.nearest_y = vector.zeros(maxOb);
obs.nearest_dist = vector.zeros(maxOb);


uOdometry0 = vector.new({0, 0, 0});

odomScale = Config.walk.odomScale or Config.world.odomScale;
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

function reset_map()
  print('Reset Occupancy Map and Robot Odometry')
  OccMap.reset();
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

function get_obstacle_info()
--  print('try find obstacle in occmap'); 
  start = unix.time();
  obstacle = OccMap.get_obstacle();
  obs.num = obstacle[1];
  for i = 1 , obs.num do
    obs.centroid_x[i] = obstacle[i + 1].centroid[1];
    obs.centroid_y[i] = obstacle[i + 1].centroid[2];
    obs.left_range[i] = obstacle[i + 1].angle_range[1];
    obs.right_range[i] = obstacle[i + 1].angle_range[2];
    obs.nearest_x[i] = obstacle[i + 1].nearest[1];
    obs.nearest_y[i] = obstacle[i + 1].nearest[2];
    obs.nearest_dist[i] = obstacle[i + 1].nearest[3];
  end

end

counter = 0;
function update()
  counter = counter + 1;
--  velocity_update();


  -- Time decay
  local time = unix.time();
  OccMap.time_decay(time);

	-- Vision Update
  vision_update();

	-- Odometry Update
  odom_update();

--  local get_obstacle = ocm.get_occ_get_obstacle();
--  if get_obstacle == 1 then
--  if counter == 25 then
    get_obstacle_info();
--    counter = 0;
--  end
--    print("get obstacles from occmap");
--    ocm.set_occ_get_obstacle(0);
--  end

  update_shm()

end

function update_shm()
	-- copy odometry to shm 
  odom = OccMap.retrieve_odometry();
  ocm.set_occ_odom(vector.new({odom.x, odom.y, odom.a}));
--  print('odom from map',odom.x..' '..odom.y..' '..odom.a);

  -- copy occmap to shm
	occmap = OccMap.retrieve_map();
	ocm.set_occ_map(occmap);		

  -- copy obstacle info to shm
  ocm.set_obstacle_num(obs.num);
  ocm.set_obstacle_cx(obs.centroid_x);
  ocm.set_obstacle_cy(obs.centroid_y);
  ocm.set_obstacle_la(obs.left_range);
  ocm.set_obstacle_ra(obs.right_range);
  ocm.set_obstacle_nx(obs.nearest_x);
  ocm.set_obstacle_ny(obs.nearest_y);
  ocm.set_obstacle_ndist(obs.nearest_dist);

end

function exit()
end
