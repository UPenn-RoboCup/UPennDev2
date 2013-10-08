function update_mesh(lidar_type)
  global POSE HEAD_LIDAR CHEST_LIDAR SLAM
  
  % Make sure the ranges matrix is in such a fashion
  % that scanlines are verticle
  if lidar_type==0 
    lidar = HEAD_LIDAR;
    lidar.ranges = lidar.ranges';
  else
    lidar = CHEST_LIDAR;
  end

  lidar.lidarrange = lidar.depths(2)-lidar.depths(1);

% Convert ranges from 0-255 to meters
t0 = tic;

  % Ensure the dimensions match
  nrays = size(lidar.ranges, 1);
  nscanlines = size(lidar.ranges, 2);
  lidar.fov_angles = lidar.fov_angles(1:nrays)';
  lidar.scanline_angles = lidar.scanline_angles(1:nscanlines);
  
  if iscell(lidar.scanline_angles)
    lidarangles=cell2mat(lidar.scanline_angles);
  else
    lidarangles=lidar.scanline_angles;
  end
  
  % Scanline angles should be a column vector
%   lidarangles = lidarangles';

  %Calculate actual ranges
  range_actual = double(lidar.ranges)/ 256 * lidar.lidarrange;
  range_actual = posz_filter(range_actual, 0.8, 0.10); %%%%%????
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  lidar.range_actual = range_actual;

  % Skip rays to speed up
  xskip=2;
  yskip=2;

  xskip=1;
  yskip=1;

  fov_angles_skipped = lidar.fov_angles(1:yskip:size(lidar.fov_angles,1), :);
  lidarangles_skipped = lidarangles(:, 1:xskip:size(lidarangles,2));
  range_skipped =range_actual(1:yskip:size(range_actual,1),1:xskip:size(range_actual,2));

  connect_th = tan(70*pi/180);
  connect_th = tan(80*pi/180);
  connect_th = tan(85*pi/180);

  max_dist = lidar.lidarrange * 0.9;
  ground_height = -0.7;
  max_height = 1.0;



  if lidar_type==0
    [verts faces cdatas facecount]=lidartrans('headmesh',...
     fov_angles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height);
  else
    [verts faces cdatas facecount]=lidartrans('chestmesh',...
     fov_angles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height);
  end
  verts=verts';



  %include pose for drawing patches
  % Would it make more sense to have the 
  % 3D mesh in body frame rather than world frame?

  ca=cos(POSE.pose_slam(3));
  sa=sin(POSE.pose_slam(3));

  vertx = verts(:,1)*ca - verts(:,2)*sa + POSE.pose_slam(1);
  verty = verts(:,1)*sa + verts(:,2)*ca + POSE.pose_slam(2);
  vertz = verts(:,3);
  
%   % Yaw
%   ca = cos(SLAM.slam_pose(3));
%   sa = sin(SLAM.slam_pose(3));
%   vertx = verts(:,1)*ca - verts(:,2)*sa + SLAM.slam_pose(1);
%   verty = verts(:,1)*sa + verts(:,2)*ca + SLAM.slam_pose(2);
%   vertz = verts(:,3);
%   
  % Pitch
  ca = cos(SLAM.torso_tilt);
  sa = sin(SLAM.torso_tilt);
  vertx = vertx*ca + vertz*sa;
  vertz = -vertx*sa + vertz*ca;
  
  vert_transformed = [vertx verty vertz];


  faces=faces(:,1:facecount)';
  cdatas=cdatas(:,1:facecount)';
  
  if lidar_type == 0
      HEAD_LIDAR.verts = vert_transformed;
      HEAD_LIDAR.faces = faces;
      HEAD_LIDAR.cdatas = cdatas;
  else
      CHEST_LIDAR.verts = vert_transformed;
      CHEST_LIDAR.faces = faces;
      CHEST_LIDAR.cdatas = cdatas;
  end

tPassed=  toc(t0);
end