function ret=waypointbody()
  %Now we use a single waypoint struct to update both waypoints
  global WAYPOINTS SLAM LIDAR POSE

  WAYPOINTS=[];
  WAYPOINTS.waypoints_xy=[];

  WAYPOINTS.init = @init;
  WAYPOINTS.add_waypoint = @add_waypoint;
  WAYPOINTS.get_waypoints = @get_waypoints;
  WAYPOINTS.get_double_approach = @get_double_approach;
  WAYPOINTS.clear_waypoints = @clear_waypoints;

  ret = WAYPOINTS;

  function init(h_text)
    WAYPOINTS.waypoints_xy=[];    
  end

  function add_waypoint(xy)
    WAYPOINTS.waypoints_xy=[WAYPOINTS.waypoints_xy;xy];
    SLAM.update_waypoints(WAYPOINTS.waypoints_xy);
    LIDAR.update_waypoints(WAYPOINTS.waypoints_xy);
    fprintf('Adding waypoint %.2f, %.2f\n',xy(1),xy(2));
  end

  function clear_waypoints(h_omap, ~, flags)
    WAYPOINTS.waypoints_xy=[];  
    SLAM.update_waypoints([]);
    LIDAR.update_waypoints([]);
  end

  function ret = get_waypoints()
    yaws = [];
    if size(WAYPOINTS.waypoints_xy,1)>1
      dys = WAYPOINTS.waypoints_xy(2:end,2) - WAYPOINTS.waypoints_xy(1:end-1,2);
      dxs = WAYPOINTS.waypoints_xy(2:end,1) - WAYPOINTS.waypoints_xy(1:end-1,1);
      yaws = atan2(dys, dxs);  %TODO: mod this angle
    end
    first_yaw = atan2(WAYPOINTS.waypoints_xy(1,2) -  POSE.pose(2),...
                WAYPOINTS.waypoints_xy(1,1) - POSE.pose(1));
    yaws = [first_yaw ;yaws];
    ret=[WAYPOINTS.waypoints_xy yaws];
  end

  function targetwp = get_double_approach()
      points2d = WAYPOINTS.waypoints_xy;
      targetwp=[];
      if numel(points2d)==4
          leftpos = points2d(1, :);
          rightpos = points2d(2, :);
          if leftpos(1)>0.30 && rightpos(1)>0.30
              centerpos = (leftpos + rightpos)/2;
              centerpos(1) = centerpos(1) - 0.3;
              centerpos(2) = centerpos(2) - 0.05; % TODO: tune
              angle = atan2(leftpos(2)-rightpos(2),leftpos(1)-rightpos(1))-pi/2;
              targetwp = [centerpos(1), centerpos(2), angle];
          end
      end      
  end
end