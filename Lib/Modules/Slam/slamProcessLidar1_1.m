%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lidar1 message handler (vertical lidar)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamProcessLidar1_1(data,name)
global SLAM LIDAR1 SERVO1 OMAP CMAP EMAP POSE IMU DVMAP

if ~isempty(data)
  LIDAR1.scan = MagicLidarScanSerializer('deserialize',data);
else
  return;
end

%make sure we have fresh data
if (CheckImu() ~= 1), return; end
if (CheckServo1() ~= 1), return; end
servoAngle = SERVO1.data.position + SERVO1.offsetYaw;
Tservo1 = trans([SERVO1.offsetx SERVO1.offsety SERVO1.offsetz])*rotz(servoAngle);
Tlidar1 = trans([LIDAR1.offsetx LIDAR1.offsety LIDAR1.offsetz]) * ...
          rotx(pi/2);
Timu = roty(IMU.data.pitch)*rotx(IMU.data.roll);
Tpos = trans([SLAM.x SLAM.y SLAM.z])*rotz(SLAM.yaw);

T = (Tpos*Timu*Tservo1*Tlidar1);
        
nStart = 250; %throw out points that pick up our own body
ranges = double(LIDAR1.scan.ranges); %convert from float to double
indGood = ranges >0.25;
indGood(1:nStart-1) = 0;

rangesGood = ranges(indGood);

xs = rangesGood.*LIDAR1.cosines(indGood);
ys = rangesGood.*LIDAR1.sines(indGood);
zs = zeros(size(xs));

xsg=xs;
ysg=ys;
zsg=zs;
onesg=ones(size(xsg));

%apply the transformation given current roll and pitch

X = [xsg; ysg; zsg; onesg];
Y=T*X;

%in body
xss = Y(1,:);
yss = Y(2,:);
zss = Y(3,:);
onez = ones(size(xss));

LIDAR1.xs = xss;
LIDAR1.ys = yss;

%publishVisPointCloud('lidar1map',xss,yss,zss);

xis = ceil((xss - OMAP.xmin) * OMAP.invRes);
yis = ceil((yss - OMAP.ymin) * OMAP.invRes);

zmax = 0.8;

indGood = (xis > 1) & (yis > 1) & (xis < OMAP.map.sizex) & (yis < OMAP.map.sizey) & (zss<zmax);
inds = sub2ind(size(OMAP.map.data),xis(indGood),yis(indGood));

cellChange = diff(inds);
newCellLogic = cellChange ~=0;
cellChangeInds  = [1 find(newCellLogic)];

nCells = length(cellChangeInds);
nCounts = zeros(1,length(inds));

xsGood = xss(indGood);
ysGood = yss(indGood);
zsGood = zss(indGood);
rsGood = rangesGood(indGood);


indsBadLogic = logical(zeros(size(inds)));

zMinPrev = 0;
zMaxPrev = 0;
xPrev = 0;
yPrev = 0;

angles = zeros(1,nCells-1);

inc=1;
for ii=1:nCells-inc
    zsc = zsGood(cellChangeInds(ii):cellChangeInds(ii+inc));
    nCounts(cellChangeInds(ii):cellChangeInds(ii+inc)) = cellChangeInds(ii+inc)-cellChangeInds(ii);
    minCurr = min(zsc);
    maxCurr = max(zsc);
    minMax = abs(maxCurr - minCurr);
    
    if (minMax > 0.06)
        indsBadLogic(cellChangeInds(ii):cellChangeInds(ii+inc)) = 1;
    end
    
    
    xCurr  = xsGood(cellChangeInds(ii));
    yCurr  = ysGood(cellChangeInds(ii));
    rCurr  = rsGood(cellChangeInds(ii));
    
    if (ii>1)
        z1 = abs(maxCurr-zMinPrev);
        z2 = abs(zMaxPrev-minCurr);
        vert = max([z1,z2]);
        dist = norm([xCurr-xPrev; yCurr-yPrev]);
        angle = atan2(vert,dist);
        dr = abs(rCurr-rPrev);
        
        %angles(ii) = vert; %angle/pi*180;
        
        if ((dr > 0.07) && (abs(angle) > 20/180*pi))
            indsBadLogic(cellChangeInds(ii):cellChangeInds(ii+inc)) = 1;
        end
    end
    
    zMinPrev = minCurr;
    zMaxPrev = maxCurr;
    xPrev    = xCurr;
    yPrev    = yCurr;
    rPrev    = rCurr;
end

%plot(angles); drawnow;

indsBad = inds(indsBadLogic);
%indsBad = inds(indsBadLogic);

%CMAP.map.data(indsBad) = CMAP.map.data(indsBad) + SLAM.cMapIncObs;
firstBad = find(indsBadLogic,1);

indsGoodLogicInds = 1:firstBad-1;
indsGood = inds(indsGoodLogicInds);

%dzs = [diff(zss) 0];
%indsBad = abs(dzs) > 0.05;
%indsBad = zss(indGood) > 0.05;
CMAP.map.data(indsBad) = CMAP.map.data(indsBad) + nCounts(indsBadLogic).*SLAM.cMapIncObs;
CMAP.map.data(indsGood) = CMAP.map.data(indsGood) + nCounts(indsGoodLogicInds)*SLAM.cMapIncFree;

%czs = ones(size(dzs)) * SLAM.cMapIncFree;
%czs = zeros(size(dzs));
%czs(1:firstBad-1) = SLAM.cMapIncFree;
%CMAP.map.data(inds)=CMAP.map.data(inds)+czs;

%czs(indsBad) = SLAM.cMapIncObs;

%CMAP.map.data(inds)=CMAP.map.data(inds)+czs;

%make sure that the costs stay bounded
tooLarge = CMAP.map.data(inds) > SLAM.maxCost;
tooSmall = CMAP.map.data(inds) < SLAM.minCost;
CMAP.map.data(inds(tooLarge)) = SLAM.maxCost;
CMAP.map.data(inds(tooSmall)) = SLAM.minCost;

%mark the cells as modified
DVMAP.map.data(indsBad)  = 1;
DVMAP.map.data(indsGood) = 1;