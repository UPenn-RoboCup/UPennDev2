function PublishMapsToMotionPlanner
global CMAP ROBOT

if(isempty(ROBOT.pose))
  return;
end

%make 3 maps for the exploration planner
%{
imagesc(CMAP.map.data);
temp = 100;
pos_y = (ROBOT.pose.data.x-CMAP.xmin)/CMAP.res;
pos_x = (ROBOT.pose.data.y-CMAP.xmin)/CMAP.res;
axis xy equal;
axis([pos_x-temp pos_x+temp pos_y-temp pos_y+temp]);
%}

costmap = CMAP.map.data;
obs_thresh = 50;
costmap(costmap>obs_thresh)=250;
costmap(costmap<=obs_thresh)=0;

%send full update
full_update.timestamp = GetUnixTime();
full_update.sent_cost_x = size(costmap,1);
full_update.sent_cost_y = size(costmap,2);
full_update.sent_elev_x = 1;
full_update.sent_elev_y = 1;
full_update.sent_cover_x = 1;
full_update.sent_cover_y = 1;
full_update.cost_map = uint8(costmap);
full_update.elev_map = int16(0);
full_update.coverage_map = uint8(0);
full_update.UTM_x = CMAP.xmin;
full_update.UTM_y = CMAP.ymin;
ipcAPIPublishVC('Lattice_Planner_Full_Update',MagicGP_FULL_UPDATESerializer('serialize',full_update));

%send pose
position_update.timestamp = GetUnixTime();
%position_update.x = ceil((SLAM.x-OMAP.xmin)*OMAP.invRes);
%position_update.y = ceil((SLAM.y-OMAP.ymin)*OMAP.invRes);
position_update.x = ROBOT.pose.data.x;
position_update.y = ROBOT.pose.data.y;
position_update.theta = ROBOT.pose.data.yaw;
ipcAPIPublishVC('Lattice_Planner_Position_Update',MagicGP_POSITION_UPDATESerializer('serialize',position_update));

