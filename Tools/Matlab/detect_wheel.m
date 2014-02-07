function ret = detect_wheel(lidar)
  connect_th = tan(50*pi/180);
  max_dist = lidar.lidarrange * 0.9;
  ground_height = -0.7;
  max_height = 2.0;
  ground_height = -0.7;

  range_actual = lidar.range_actual;
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);
  range_actual = posz_filter(range_actual, 0.8, 0.10);

	[verts faces cdatas facecount]=lidartrans('headmesh',...
     lidar.rayangles, lidar.lidarangles_trimmed, lidar.range_actual, ...
     connect_th, max_dist, ground_height, max_height);

  lidar.verts0 = verts;  

  m=size(lidar.rayangles,1)
  n=size(lidar.lidarangles_trimmed,2)

  xpos = reshape(verts(1,:),[m n]);
  ypos = reshape(verts(2,:),[m n]);
  zpos = reshape(verts(3,:),[m n]);


  rmin = 0.5;

  xpos(find(xpos<0.2)) = 2.5;
  xpos(find(xpos>0.5)) = 2.5;
%  xpos(find(xpos.^2 + ypos.^2 + zpos.^2 > rmin*rmin)) = 2.5;

%{
  l_line=[];
  r_line=[];

  for i=1:size(range_actual,1)
    scanline = xpos(i,:);
    f = find(scanline<2.5);
    l0 = min(f);
    r0 = max(f);
    if numel(l0)>0
      l_line=[l_line;[xpos(i,l0) ypos(i,l0) zpos(i,l0)]];
      r_line=[r_line;[xpos(i,r0) ypos(i,r0) zpos(i,r0)]];
    end
  end

close all;

figure(1);
subplot(1,2,1);
hold on;
  plot3(l_line(:,1),l_line(:,2),l_line(:,3))
  plot3(r_line(:,1),r_line(:,2),r_line(:,3))
%  plot(r_line);
hold off;

subplot(1,2,2);
  surf(2.5-xpos);
colormap(HOT);


%}

  ret.xpos = xpos;
  ret.ypos = ypos;
  ret.zpos = zpos;

  wheel_scanline(ret);
end
