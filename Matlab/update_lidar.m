%% Updates 2D depth image and 3D mesh from lidar data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Apply pose at the time of capture
%  lidar.posex1 = lidar.posex(1:linenum);
%  lidar.posey1 = lidar.posey(1:linenum);
%  lidar.posea1 = lidar.posea(1:linenum);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ nBytes ] = update_lidar(lidar_type)
global POSE HEAD_LIDAR CHEST_LIDAR LIDAR
t0 = tic;

if lidar_type==0
    lidar = HEAD_LIDAR;
    linenum = size(lidar.ranges, 1)
else
    lidar = CHEST_LIDAR;
    linenum = size(lidar.ranges, 2)
end

lidarangles = lidar.lidarangles;

if lidar.p1~=0
    jStart = lidarangles(1);
    jEnd = lidarangles(linenum);
    if (jStart>jEnd && lidar.type==0) || (jStart<jEnd && lidar.type==1)
        depthfig = flipdim(lidar.ranges,1);
    else
        depthfig = lidar.ranges;
    end
    
    if lidar.type==0 % head
        disp('HEAD LIDARDATA')%Update 2D depth image
        depthfig = flipdim(depthfig,2);
        set(lidar.h1,'Cdata', depthfig);
        set(lidar.p1, 'XLim', [1 size(lidar.ranges,2)]);
        set(lidar.p1, 'YLim', [1 size(lidar.ranges,1)]);
    else %chest
        set(lidar.h1,'Cdata', depthfig');
        set(lidar.p1, 'XLim', [1 size(lidar.ranges,1)]);
        set(lidar.p1, 'YLim', [1 size(lidar.ranges,2)]);
    end
    
    LIDAR.xsize = size(lidar.ranges,2);
    LIDAR.ysize = size(lidar.ranges,1);
    LIDAR.xmag = size(lidar.ranges,2)/2;
    LIDAR.ymag = size(lidar.ranges,1)/2;
end

xskip=2;
yskip=2;
xskip=1;
yskip=1;

%Calculate angles and ranges
lidar.lidarangles_trimmed = lidarangles(1:linenum);
%lidar.rayangles = ([lidar.range0 : lidar.range1]')*0.25*pi/180;
%lidar.rayangles = [HEAD_LIDAR.range0:.25*pi/180:HEAD_LIDAR.range1]
lidar.rayangles = [lidar.range0:.25*pi/180:lidar.range1];

range_actual = double(lidar.ranges)' / 256 * lidar.lidarrange;
range_actual = posz_filter(range_actual, 0.8, 0.10);
range_actual = posz_filter(range_actual, 0.8, 0.10);
lidar.range_actual = range_actual;


if lidar_type==0
    HEAD_LIDAR = lidar;
else
    CHEST_LIDAR = lidar;
end

lidarangles = lidar.lidarangles_trimmed;
rayangles_skipped = lidar.rayangles(1:xskip:size(lidar.rayangles,1),:);
lidarangles_skipped = lidarangles(1, 1:yskip:size(lidarangles,2));
range_skipped =range_actual(1:xskip:size(range_actual,1),1:yskip:size(range_actual,2));


connect_th = tan(70*pi/180);
connect_th = tan(80*pi/180);
connect_th = tan(85*pi/180);

max_dist = lidar.lidarrange * 0.9;
ground_height = -0.7;
max_height = 1.0;



%{
  if lidar.type==0
    [verts faces cdatas facecount]=lidartrans('headmesh',...
     rayangles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height);
  else
    [verts faces cdatas facecount]=lidartrans('chestmesh',...
     rayangles_skipped, lidarangles_skipped, range_skipped, ...
     connect_th, max_dist, ground_height, max_height);
  end
  verts=verts';

  %include pose for drawing patches

  ca=cos(POSE.pose_slam(3));
  sa=sin(POSE.pose_slam(3));

  vertx = verts(:,1)*ca - verts(:,2)*sa + POSE.pose_slam(1);
  verty = verts(:,1)*sa + verts(:,2)*ca + POSE.pose_slam(2);
  vert_transformed = [vertx verty verts(:,3)];

  faces=faces(:,1:facecount)';
  cdatas=cdatas(:,1:facecount)';

  lidar.verts = vert_transformed;
  lidar.faces = faces;
  lidar.cdatas = cdatas;
  if lidar_type==0
    HEAD_LIDAR = lidar;
  else
    CHEST_LIDAR = lidar;
  end
%}

tPassed = toc(t0);
fprintf('Processed lidar: %f',tPassed)
end
