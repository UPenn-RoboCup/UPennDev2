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
  % lidarangles = lidarangles';

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

  connect_th = tan(85*pi/180);

  max_dist = lidar.lidarrange * 0.9;
  
  ground_height = -POSE.body_height + 0.1;
  max_height = -POSE.body_height + 2.0;

  if lidar_type==0
    [verts faces cdatas facecount]=lidartrans('headmeshglobal',...
     fov_angles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height, lidar.rpy,...
      lidar.poses(1,:),lidar.poses(2,:),lidar.poses(3,:));
  else
    [verts faces cdatas facecount]=lidartrans('chestmeshglobal',...
     fov_angles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height, lidar.rpy,...
      lidar.poses(1,:),lidar.poses(2,:),lidar.poses(3,:));
  end

  verts = verts';
  verts(:,3)= verts(:,3)+ POSE.body_height;
  vert_transformed = verts;

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
  
tPassed = toc(t0);
end