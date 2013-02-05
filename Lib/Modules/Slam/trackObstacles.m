%input: ranges and angles of lidar, T = transform from lidar frame to world
function obsTracks = trackObstacles(ranges,angles,T)
global TRACK
persistent cntr

if isempty(cntr), cntr = 0; end
if isempty(TRACK)
  TRACK.msgName = GetMsgName('VelTracks');
  ipcAPIDefine(TRACK.msgName);
end

obsTracks = [];

rmin = 0.3;
rmax = 25;

%calculate the clusters
clusterThreshold = 0.1;
clusterNMin      = 4;   %minimum number of points
[cistart ciend] = scanCluster(ranges,clusterThreshold,clusterNMin);

if isempty(cistart)
  return;
end

%find the middle range
imean = ceil((cistart+ciend)/2);
rmean = ranges(imean);

%approximate length (width) of the obstacle
obsLen = rmean .*(ciend-cistart)*0.25/180*pi; %s=r*theta

minLen = 0.2; %meters
maxLen = 0.7;
isizeMatch = (obsLen > minLen) & (obsLen < maxLen) & (rmean > rmin) & (rmean < rmax);


%compute centers of the clusters
[xTrack, yTrack] = clusterCenter(ranges,angles,cistart,ciend);

%sensor frame
xts = xTrack(isizeMatch);
yts = yTrack(isizeMatch);

if (length(xts) < 1)
  return;
end

X=[xts yts zeros(size(xts)) ones(size(xts))];
Y=X*(T');

xtg = Y(:,1);
ytg = Y(:,2);


obsTracks.xs  = xtg;
obsTracks.ys  = ytg;
obsTracks.vxs = zeros(size(xtg));
obsTracks.vys = zeros(size(ytg));
obsTracks.t   = GetUnixTime();
