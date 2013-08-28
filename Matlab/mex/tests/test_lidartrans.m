ray_angles= [-45:0.25:45]'*pi/180;
lidar_angles= [-90:0.25:90]*pi/180;

ray_angles= [-45:1:45]'*pi/180;
lidar_angles= [-90:1:90]*pi/180;


ray_angles= [-45:2:45]'*pi/180;
lidar_angles= [-90:2:90]*pi/180;


ray_angles= [-45:5:45]'*pi/180;
lidar_angles= [-90:5:90]*pi/180;
%{

%}


ranges = 2*ones(size(ray_angles,1) , size (lidar_angles,2) );

for i=1:size(ranges,1)
  for j=1:size(ranges,2)
    k = floor(j/size(ranges,2)*10);
    l = floor(i/size(ranges,1)*5);
    ranges(i,j) = 1.0+ k*k*0.02 + l*l*0.01;
  end
end



clf;

size(ray_angles)
size(lidar_angles)
size(ranges)

%{
tic;
[px py pz]=lidartrans('chestmap',ray_angles,lidar_angles,ranges,0.1);
%[px py pz]=lidartrans('headmap',ray_angles,lidar_angles,ranges);
toc;

plot3(px,py,pz,'.')
view(-45,45);
drawnow;

pause;

%}

clf;
tic;
%tangent value 
connect_th = tan(70*pi/180);
max_dist = 2.5;
ground_height = -0.7;
max_height = 2.0;

[verts faces cdatas facecount]=lidartrans('chestmesh',...
	ray_angles, lidar_angles,ranges,...
	connect_th, max_dist, ground_height, max_height);

%[verts faces cdatas facecount]=lidartrans('headmesh',...
%	ray_angles, lidar_angles,ranges,...
%	connect_th, max_dist, ground_height, max_height);

verts=verts';
faces=faces(:,1:facecount)';
cdatas=cdatas(:,1:facecount)';
toc;



tic;
p=patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cdatas,... 
	'FaceColor','flat','EdgeColor','none',...
        'AmbientStrength',0.4,'SpecularStrength',0.9 );
light('Position',[0 3 3]);
%lighting phong
lighting flat
view(-70,30);
axis equal
drawnow;
toc;

