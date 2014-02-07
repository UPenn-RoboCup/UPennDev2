valid_rays = find(posx(1,:)~=0);
ray_start = min(valid_rays);
ray_end = max(valid_rays);

valid_lidarindex = find(posx(:, floor((ray_start+ray_end)/2))~=0);
lidarangle_end = max(valid_lidarindex);

%Cut only valid rays
posx1 = posx(1:lidarangle_end, ray_start:ray_end);
posy1 = posy(1:lidarangle_end, ray_start:ray_end);
posz1 = posz(1:lidarangle_end, ray_start:ray_end);


ray_mid = (size(posx1,2)+1)/2;

rayangles = ((1:size(posx1,2)) - ray_mid)*0.25*pi/180;

lidarangles=[];
%{
for i=1:size(posx1,1) 
  r = sqrt(posx1(i,ray_mid)^2 + posz1(i,ray_mid)^2);
  c = posz1(i,ray_mid) - 0.30;
  %-sa*r + ca*0.10  = posz -0.30 = c
  %-sa * r + sqrt(1-sa^2)*0.10 = c
  %(c + sa*r) ^2 = 0.01 * (1-sa^2) 
  %(0.01 + r^2) sa^2 + 2*c*r *sa + c^2-0.01 =0
  param_a = 0.01+ r*r;
  param_b = c*r;
  param_c = c*c-0.01;
  x1=( -param_b +(param_b^2 - param_a*param_c) ) / param_a;
  lidarangles(i) = asin(x1);
end
%}

for i=1:size(posx1,1)
  lidarangles(i) = (-30 + 90*i/size(posx1,1))*pi/180;
end


%OFFSET
lidarangles = lidarangles + 25*pi/180;


ranges=[];
for i=1:size(posx1,1)
   for j=1:size(posx1,2)
       r=sqrt(posx1(i,j)^2+posy1(i,j)^2+posz1(i,j)^2);
       ranges(i,j)=r;
   end
end


lidaroffset = -30*pi/180;

lidar=[];
lidar.type = 0;
lidar.range0 = ray_start-541;
lidar.range1 = ray_end-541;
lidar.ranges = ranges; 
lidar.lidarrange = 256;
lidar.lidarangles = lidarangles + lidaroffset;
lidar.rayangles = rayangles;

