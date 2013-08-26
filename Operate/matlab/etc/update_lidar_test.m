addpath(genpath('.'));

close all;
addpath(genpath('mex'))

xskip=2;
yskip=2;




clf;
tic;

linenum = size(lidar.ranges,1);
filpped = 0;

%lidarangles=cell2mat(lidar.lidarangles);
lidarangles=lidar.lidarangles;

lidarangles=lidarangles(1:linenum);

jStart = lidarangles(1);
jEnd = lidarangles(linenum);
  if (jStart>jEnd && lidar.type==0) || (jStart<jEnd && lidar.type==1)
    depthfig = flipdim(lidar.ranges,1);
   else
    depthfig = lidar.ranges;
  end

  if (lidar.type==0)
    depthfig = flipdim(depthfig,2);
  end

%{
  %Update 2D depth image
  if lidar.type==0 % head
    set(lidar.h1,'Cdata', depthfig);
    set(lidar.p1, 'XLim', [1 size(lidar.ranges,2)]);
    set(lidar.p1, 'YLim', [1 size(lidar.ranges,1)]);
  else %chest
    set(lidar.h1,'Cdata', depthfig');
    set(lidar.p1, 'XLim', [1 size(lidar.ranges,1)]);
    set(lidar.p1, 'YLim', [1 size(lidar.ranges,2)]);
  end
%}
  rayangles = ([lidar.range0 : lidar.range1]')*0.25*pi/180;

  range_actual = double(lidar.ranges)' / 256 * lidar.lidarrange;

toc;





disp('filtering')
tic;
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
 
  %xskip = floor(size(rayangles,1) / 150);
  %yskip = floor(size(lidarangles,2) / 150);

  rayangles=rayangles(1:xskip:size(rayangles,1),:);
  lidarangles=lidarangles(1, 1:yskip:size(lidarangles,2));
  range_actual =range_actual(1:xskip:size(range_actual,1),1:yskip:size(range_actual,2));
toc;

connect_th = tan(50*pi/180);
connect_th = tan(70*pi/180);


  max_dist = lidar.lidarrange * 0.95;
  ground_height = -0.7;
  max_height = 2.0;
  ground_height = -0.7;


disp('projecting')
tic;
  if lidar.type==0
    [verts faces cdatas facecount]=lidartrans('headmesh',...
       rayangles, lidarangles, range_actual, ...
       connect_th, max_dist, ground_height, max_height);
  %  view(lidar.p2,-30,45)
  else
    [verts faces cdatas facecount]=lidartrans('chestmesh',...
       rayangles, lidarangles, range_actual, ...
       connect_th, max_dist, ground_height, max_height);
  %  view(lidar.p2,-30,45)
  end
  verts=verts';
  faces=faces(:,1:facecount)';
  cdatas=cdatas(:,1:facecount)';
toc;

f=figure (1);
lidar.p2 = gca;

disp('patching')
tic;
  lidar.h2 = patch('FaceColor','flat','EdgeColor','none',...
      'AmbientStrength',0.4,'SpecularStrength',0.9 );
  set(lidar.h2,'Faces',faces);
  set(lidar.h2,'Vertices',verts);
  set(lidar.h2,'FaceVertexCData',cdatas);
  set(lidar.p2, 'XLim', [-1 3]);
  set(lidar.p2, 'YLim', [-2 2]);
  set(lidar.p2, 'ZLim', [-0.7 3.3]);
  view(lidar.p2,-30,45)
  light('Position',[0 3 3]);
  lighting flat
  drawnow;
toc;

robotbody();
axis([-1 3 -2 2 -0.8 3.2]);

fps = 0;
for i=0:720
    
    t0=tic;
    view(i,30);
    drawnow;    
    t1=toc(t0);
    fps = 1/t1;
    fpsstr= sprintf('update: %.1f FPS',fps);
    disp(fpsstr)        
end


posx_filtered = verts(:,1);
posy_filtered = verts(:,2);
posz_filtered = verts(:,3);

