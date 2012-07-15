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
obs.front = 0;
obs.left = 0;
obs.right = 0;
 

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
  vel = (uOdometry - lastPos);
  ocm.set_occ_vel(vel);

  lastPos = uOdometry; 
  lastTime = curTime;
end

function get_obstacle_info()
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

function get_obstacle_dir()
  min_front_angle = Config.occ.min_front_angle or -15*math.pi/180;
  max_front_angle = Config.occ.max_front_angle or 15*math.pi/180;
  min_left_angle = Config.occ.min_left_angle or 0*math.pi/180;
  max_left_angle = Config.occ.max_left_angle or 70*math.pi/180;
  min_right_angle = Config.occ.min_right_angle or 110*math.pi/180;
  max_right_angle = Config.occ.max_right_angle or 180*math.pi/180;
  min_obstacle_range = Config.occ.min_obstacle_range or 3*math.pi/180;
  min_obstacle_distance = Config.occ.min_obstacle_distance or 0.2

  -- check front
  front_angle = {min_front_angle, max_front_angle}
  front_obs = false;

  obs_range = {180*math.pi/180, 0*math.pi/180};
  for cnt = 1 , obs.num do
    flag = true; -- assume in cone 
    -- check in front cone range
    if (obs.left_range[cnt] > front_angle[2]) or 
      (obs.right_range[cnt] < front_angle[1]) then
      --print('range fail')
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    --print((obs.right_range[cnt] - obs.left_range[cnt])*180/math.pi)
    if (obs.right_range[cnt] - obs.left_range[cnt] < min_obstacle_range) then
      --print('size fail')
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    --print(obs.nearest_dist[cnt])
    if (obs.nearest_dist[cnt] > min_obstacle_distance) then
      --print('distance fail')
      flag = flag and false;
    end

    if flag then
      obs_range[1] = math.min(obs_range[1], obs.left_range[cnt]);
      obs_range[2] = math.max(obs_range[2], obs.right_range[cnt]);
    end

    front_obs = front_obs or flag;
  end

  -- check left side obstacleleft_angle = {-45*math.pi/180, 45*math.pi/180}
  left_angle = {min_left_angle, max_left_angle}
  left_obs = false;
  for cnt = 1, obs.num do
    flag = true; -- assume in cone 
    -- check in left cone range
    --print(cnt, obs.left_range[cnt]*180/math.pi, obs.right_range[cnt]*180/math.pi)
    if (obs.left_range[cnt] > left_angle[2]) or 
      (obs.right_range[cnt] < left_angle[1]) then
      --print('range fail')
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    --print((obs.right_range[cnt] - obs.left_range[cnt])*180/math.pi)
    if (obs.right_range[cnt] - obs.left_range[cnt] < min_obstacle_range) then
      --print('size fail')
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    --print(obs.nearest_dist[cnt])
    if (obs.nearest_dist[cnt] > min_obstacle_distance) then
      --print('distance fail')
      flag = flag and false;
    end
    left_obs = left_obs or flag;
  end
  
  -- check right side obstacle
  right_angle = {min_right_angle, max_right_angle}
  right_obs = false;
  for cnt = 1, obs.num do
    flag = true; -- assume in cone 
    -- check in right cone range
    if (obs.left_range[cnt] > right_angle[2]) or 
      (obs.right_range[cnt] < right_angle[1]) then
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    if (obs.right_range[cnt] - obs.left_range[cnt] < min_obstacle_range) then
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    if (obs.nearest_dist[cnt] > min_obstacle_distance) then
      flag = flag and false;
    end
    right_obs = right_obs or flag;
  end

  if front_obs then obs.front = 1; else obs.front = 0; end
  if left_obs then obs.left = 1; else obs.left = 0; end
  if right_obs then obs.right = 1; else obs.right = 0; end
   
  return obs;

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

  if counter == 25 then
    get_obstacle_info();
  
    get_obstacle_dir();
    counter = 0;
  end

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

  ocm.set_obstacle_front(obs.front);
  ocm.set_obstacle_left(obs.left);
  ocm.set_obstacle_right(obs.right);

end

function exit()

end
