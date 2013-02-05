

nyaw2 = 21;
dyaw2 = 0.1/180*pi;
nxs2  = 1;
nys2  = 1;
dx2   = 0.01;
dy2   = 0.01;


yawRange2 = floor(nyaw2/2);
xRange2   = floor(nxs2/2);
yRange2   = floor(nys2/2);

%create the candidate locations in each dimension
aCand2 = (-yawRange2:yawRange2)*dyaw2+SLAM.yaw;
xCand2 = (-xRange2:xRange2)*dx2+SLAM.x;
yCand2 = (-yRange2:yRange2)*dy2+SLAM.y;

%get a local 3D sampling of pose likelihood
hits = ScanMatch2D('match',OMAP.map.data,xsss,ysss, ...
  xCand2,yCand2,aCand2);

%find maximum
[hmax imax] = max(hits(:));
[kmax mmax jmax] = ind2sub([nxs2,nys2,nyaw2],imax);

SLAM.yaw = aCand2(jmax);



