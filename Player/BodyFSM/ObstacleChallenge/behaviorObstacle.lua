module(..., package.seeall);

require('ocm')

function check_obstacle(vStep)
  obs = {}
  obs_num = ocm.get_obstacle_num()
  obs_centroid_x = ocm.get_obstacle_cx();
  obs_centroid_y = ocm.get_obstacle_cy();
  obs_left_range = ocm.get_obstacle_la();
  obs_right_range = ocm.get_obstacle_ra();
  obs_nearest_x = ocm.get_obstacle_nx();
  obs_nearest_y = ocm.get_obstacle_ny();
  obs_nearest_dist = ocm.get_obstacle_ndist();

  -- check front
  front_angle = {(90-15)*math.pi/180, (90+15)*math.pi/180}
  front_obs = false;

  obs_range = {180*math.pi/180, 0*math.pi/180};
  for cnt = 1 , obs_num do
    flag = true; -- assume in cone 
    -- check in front cone range
--    print(obs_left_range[cnt]*180/math.pi, obs_right_range[cnt]*180/math.pi)
    if (obs_left_range[cnt] > front_angle[2]) or 
      (obs_right_range[cnt] < front_angle[1]) then
--      print('range fail')
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    minObstacleSize = 3*math.pi/180;
--    print((obs_right_range[cnt] - obs_left_range[cnt])*180/math.pi)
    if (obs_right_range[cnt] - obs_left_range[cnt] < minObstacleSize) then
--      print('size fail')
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    minObsDistance = 0.2;
--    print(obs_nearest_dist[cnt])
    if (obs_nearest_dist[cnt] > minObsDistance) then
--      print('distance fail')
      flag = flag and false;
    end

    if flag then
      obs_range[1] = math.min(obs_range[1], obs_left_range[cnt]);
      obs_range[2] = math.max(obs_range[2], obs_right_range[cnt]);
    end

    front_obs = front_obs or flag;
  end

  -- check left side obstacleleft_angle = {-45*math.pi/180, 45*math.pi/180}
  left_angle = {0*math.pi/180, 70*math.pi/180}
  for cnt = 1, obs_num do
    flag = true; -- assume in cone 
    -- check in left cone range
--    print(cnt, obs_left_range[cnt]*180/math.pi, obs_right_range[cnt]*180/math.pi)
    if (obs_left_range[cnt] > left_angle[2]) or 
      (obs_right_range[cnt] < left_angle[1]) then
--      print('range fail')
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    minObstacleSize = 3*math.pi/180;
--    print((obs_right_range[cnt] - obs_left_range[cnt])*180/math.pi)
    if (obs_right_range[cnt] - obs_left_range[cnt] < minObstacleSize) then
--      print('size fail')
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    minObsDistance = 0.2;
--    print(obs_nearest_dist[cnt])
    if (obs_nearest_dist[cnt] > minObsDistance) then
--      print('distance fail')
      flag = flag and false;
    end
    left_obs = left_obs or flag;
  end
  
  -- check right side obstacle
  right_angle = {110*math.pi/180, 180*math.pi/180}
  right_obs = false;
  for cnt = 1, obs_num do
    flag = true; -- assume in cone 
    -- check in right cone range
    if (obs_left_range[cnt] > right_angle[2]) or 
      (obs_right_range[cnt] < right_angle[1]) then
      flag = flag and false;
    end
    -- check obstacle blob size in terms of angle range
    minObstacleSize = 3*math.pi/180;
    if (obs_right_range[cnt] - obs_left_range[cnt] < minObstacleSize) then
      flag = flag and false;
    end
    -- check distance for nearest poinpt of the blob
    minObsDistance = 0.2;
    if (obs_nearest_dist[cnt] > minObsDistance) then
      flag = flag and false;
    end
    right_obs = right_obs or flag;
  end

  
  obs.vStep = vStep
--[[
  if front_obs then
    obs.vStep[1] = obs.vStep[1] - 0.01;
    if (math.pi/2 - obs_range[1]) > (obs_range[2] - math.pi/2) then
      obs.vStep[2] = obs.vStep[2] - 0.03;
    else
      obs.vStep[2] = obs.vStep[2] + 0.03;
    end
  end
--]]
  
  obs.front = front_obs;
  obs.front_range = obs_range;
  obs.left = left_obs;
  obs.right = right_obs;
   
  return obs;
end


function avoid(obs, vStep)
end
