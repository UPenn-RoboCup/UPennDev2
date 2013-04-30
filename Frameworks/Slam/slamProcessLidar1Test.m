%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lidar1 message handler (vertical lidar)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamProcessLidar1(data,name)
global SLAM LIDAR1 SERVO1 OMAP CMAP EMAP POSE IMU

if ~CheckImu(),
  disp('Waiting for IMU...');
end

lidar1Position = [0.2 0 0.47]; % Lidar1 position
xBinLength = 0.05; % meters
xBinMax = round((25.0)/xBinLength);
xBin = xBinLength*[1:xBinMax];

if ~isempty(data)
  LIDAR1.scan = MagicLidarScanSerializer('deserialize',data);
else
  return;
end

%make sure we have fresh data
if (CheckImu() ~= 1), return; end
Timu = roty(IMU.data.pitch)*rotx(IMU.data.roll);


if (CheckServo1() ~= 1), return; end
%servoAngle = SERVO1.data.position + 3/180*pi;
servoAngle = SERVO1.data.position;
%Tservo1 = trans([SERVO1.offsetx SERVO1.offsety SERVO1.offsetz])*rotz(servoAngle);
Tservo = rotz(servoAngle);
%{
Tlidar1 = trans([LIDAR1.offsetx LIDAR1.offsety LIDAR1.offsetz]) * ...
          rotx(pi/2);
Timu = roty(IMU.data.pitch)*rotx(IMU.data.roll);
Tpos = trans([SLAM.x SLAM.y SLAM.z])*rotz(SLAM.yaw);

T = (Tpos*Timu*Tservo1*Tlidar1);
 %}


ranges = double(LIDAR1.scan.ranges); %convert from float to double

pRaw = ranges.*LIDAR1.cosines + lidar1Position(1);
pRaw(3,:) = ranges.*LIDAR1.sines + lidar1Position(3);
pRaw(4,:) = 1;

pRot = (Timu*Tservo)*pRaw;
rLidar = sqrt(pRot(1,:).^2+pRot(2,:).^2);
zLidar = pRot(3,:);

% Ignore inside robot, and too high:
  indClip = find((ranges > 0.3) & (pRot(1,:) > 0.25) & (zLidar < 1.5));
if isempty(indClip), return, end

xClip = rLidar(indClip);
zClip = zLidar(indClip);

% Detect negative cliffs:
zDiff = zClip([2:end end]) - zClip;
xDiff = xClip([2:end end]) - xClip;
iCliff = find(zDiff < min(-0.04, -tand(15)*xDiff));
xCliff = xClip(iCliff);
zCliff = zClip(iCliff);

plot(xClip, zClip, '-');
hold on;

stats = binStats(xClip./xBinLength, zClip, xBinMax);

SLAM.lidar1Stats = stats;

counts = [stats.count];
zMean = [stats.mean];
zMaxMin = [stats.max] - [stats.min];

iGnd = find((counts >= 3) & (zMaxMin < 0.05) & ...
	    (zMean < 0.3*xBin + 0.05) & (zMean > -0.3*xBin -0.05));

iObs = find((zMaxMin > 0.06) | (zMean > 0.4*xBin + 0.10));

plot(xBin(iGnd), zMean(iGnd),'go', ...
     xBin(iObs), zMean(iObs), 'rx', ...
     xCliff, zCliff, 'r*');

hold off;
drawnow
