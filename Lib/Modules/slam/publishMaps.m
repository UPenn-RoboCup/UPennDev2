function publishMaps(cmap,emap,pose)

persistent i initialized

if isempty(i)
  i=0;
end

mapRes = 0.1;

i=i+1;

if isempty(initialized)
  %initialize IPC
  ipcAPIDefine('Global Planner Map Initialization',MagicGP_MAP_DATASerializer('getFormat'));
  ipcAPIDefine('Global Planner Robot Parameters',MagicGP_ROBOT_PARAMETERSerializer('getFormat'));
  ipcAPIDefine('Global Planner Full Update',MagicGP_FULL_UPDATESerializer('getFormat'));
  ipcAPIDefine('Global Planner Position Update',MagicGP_POSITION_UPDATESerializer('getFormat'));

  %After this you need to initialize Jon's planner.  This only needs to be done once.

  %initialze map over IPC
  map_init.timestamp = i;
  map_init.cost_size_x = 10;
  map_init.cost_size_y = 10;
  map_init.elev_size_x = 10;
  map_init.elev_size_y = 10;
  map_init.coverage_size_x = 10;
  map_init.coverage_size_y = 10;
  map_init.cost_cell_size = mapRes;
  map_init.elev_cell_size = mapRes;
  map_init.coverage_cell_size = mapRes;
  ipcAPIPublishVC('Global Planner Map Initialization',MagicGP_MAP_DATASerializer('serialize',map_init));

  %send robot parameters over IPC
  robot_params.MAX_VELOCITY = 1.0;
  robot_params.MAX_TURN_RATE = pi/2;
  robot_params.I_DIMENSION = 4;
  robot_params.J_DIMENSION = 2;
  robot_params.sensor_radius = 30;
  robot_params.sensor_height = 62.8;
  robot_params.PerimeterArray = [1;1;1;-1;-1;-1;-1;1]*0.25;
  ipcAPIPublishVC('Global Planner Robot Parameters',MagicGP_ROBOT_PARAMETERSerializer('serialize',robot_params));
  
  initialized =1;
end

%This next bit needs to be done every few seconds to get a new path
%from Jon's planner.  My map has positive values being obstacles,
%negative values being free space and 0 being unknown.

%make 3 maps for the exploration planner
costmap = cmap;
costmap(costmap>50)=250;
costmap(costmap<=50)=0;
elevmap = costmap*4;
covmap = emap;
%covmap(covmap~=0) = 249;

%send full update
full_update.timestamp = i;
full_update.sent_cost_x = size(cmap,1);
full_update.sent_cost_y = size(cmap,2);
full_update.sent_elev_x = size(cmap,1);
full_update.sent_elev_y = size(cmap,2);
full_update.sent_cover_x = size(cmap,1);
full_update.sent_cover_y = size(cmap,2);
full_update.cost_map = uint8(costmap');
full_update.elev_map = int16(elevmap');
full_update.coverage_map = uint8(covmap');
ipcAPIPublishVC('Global Planner Full Update',MagicGP_FULL_UPDATESerializer('serialize',full_update));

%send position
position_update.timestamp = i;
position_update.x = pose.x*mapRes;
position_update.y = pose.y*mapRes;
position_update.theta = pose.theta;
ipcAPIPublishVC('Global Planner Position Update',MagicGP_POSITION_UPDATESerializer('serialize',position_update));