module(..., package.seeall);

require('ocm')

function check_obstacle(vStep)
  obs_num = ocm.get_obstacle_num()
  obs_centroid_x = ocm.get_obstacle_cx();
  obs_centroid_y = ocm.get_obstacle_cy();
  obs_left_range = ocm.get_obstacle_la();
  obs_right_range = ocm.get_obstacle_ra();
  obs_nearest_x = ocm.get_obstacle_nx();
  obs_nearest_y = ocm.get_obstacle_ny();
  obs_nearest_dist = ocm.get_obstacle_ndist();

  -- check front
  front_angle = {(90-20)*math.pi/180, (90+20)*math.pi/180}
  front_obs = false;

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
    -- check distance for nearest point of the blob
    minObsDistance = 0.3;
--    print(obs_nearest_dist[cnt])
    if (obs_nearest_dist[cnt] > minObsDistance) then
--      print('distance fail')
      flag = flag and false;
    end

    front_obs = front_obs or flag;
  end

  return front_obs;
end
