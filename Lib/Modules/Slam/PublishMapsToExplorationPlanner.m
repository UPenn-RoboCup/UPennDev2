function PublishMapsToExplorationPlanner
global SLAM OMAP EMAP

%make 3 maps for the exploration planner
costmap = OMAP.map.data;
costmap(costmap>50)=250;
costmap(costmap<=50)=0;
elevmap = costmap*4;
covmap = EMAP.map.data;
%covmap(covmap~=0) = 249;
%{
    imagesc(costmap');
    colormap gray;
    hold on;
    tempX = (SLAM.x-EMAP.xmin)*EMAP.invRes;
    tempY = (SLAM.y-EMAP.ymin)*EMAP.invRes;
    plot(tempX,tempY,'bx');
    temp = 20*EMAP.invRes;
    axis([tempX-temp tempX+temp tempY-temp tempY+temp]);
    axis xy;
    drawnow;
%}
%send full update
full_update.timestamp = GetUnixTime();
full_update.sent_cost_x = size(costmap,1);
full_update.sent_cost_y = size(costmap,2);
full_update.sent_elev_x = size(elevmap,1);
full_update.sent_elev_y = size(elevmap,2);
full_update.sent_cover_x = size(covmap,1);
full_update.sent_cover_y = size(covmap,2);
full_update.cost_map = uint8(costmap);
full_update.elev_map = int16(elevmap);
full_update.coverage_map = uint8(covmap);
full_update.UTM_x = OMAP.xmin;
full_update.UTM_y = OMAP.ymin;
ipcAPIPublishVC('Global Planner Full Update',MagicGP_FULL_UPDATESerializer('serialize',full_update));

%send pose
position_update.timestamp = GetUnixTime();
%position_update.x = ceil((SLAM.x-OMAP.xmin)*OMAP.invRes);
%position_update.y = ceil((SLAM.y-OMAP.ymin)*OMAP.invRes);
position_update.x = SLAM.x;
position_update.y = SLAM.y;
position_update.theta = SLAM.yaw;
ipcAPIPublishVC('Global Planner Position Update',MagicGP_POSITION_UPDATESerializer('serialize',position_update));

