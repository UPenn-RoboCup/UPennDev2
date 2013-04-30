function plannerIncMap
clear all;

global POSE ROBOT

SetMagicPaths;

poseInit;
initMapProps;
cmapInit;
omapInit;
emapInit;

ipcInit;
ipcReceiveSetFcn(POSE.msgName,        @PoseMsgHandler);
ipcAPISetMsgQueueLength(POSE.msgName, 5);
ipcReceiveSetFcn(GetMsgName('IncMapUpdateH'),  @MapUpdateMsgHandlerH);
ipcAPISetMsgQueueLength(GetMsgName('IncMapUpdateH'), 5);
ipcReceiveSetFcn(GetMsgName('IncMapUpdateV'),  @MapUpdateMsgHandlerV);
ipcAPISetMsgQueueLength(GetMsgName('IncMapUpdateV'), 5);


DefinePlannerMessages;

POSE.cntr =1;
ROBOT.pose = [];

plannerUpdatePeriod = 3;
lastPlannerUpdate = GetUnixTime();
while(1)
  ipcReceiveMessages(100);

  if (GetUnixTime() - lastPlannerUpdate > plannerUpdatePeriod)
    PublishMapsToMotionPlanner;
    fprintf('sent planner map\n');
    lastPlannerUpdate = GetUnixTime();
  end

  pause(0.1);
end


function PoseMsgHandler(data,name)
global ROBOT
  if isempty(data)
    return;
  end

  ROBOT.pose.data = MagicPoseSerializer('deserialize',data);
  
  %fprintf(1,'got pose update\n');

function MapUpdateMsgHandlerV(data,name)
global CMAP;

  if isempty(data)
    return
  end
  
  msgSize = length(data);
  fprintf(1,'got vertical map update\n');
  
  update = deserialize(data);
  
  xis = ceil((update.xs - CMAP.xmin) * CMAP.invRes);
  yis = ceil((update.ys - CMAP.ymin) * CMAP.invRes);

  indGood = (xis > 1) & (yis > 1) & (xis < CMAP.map.sizex) & (yis < CMAP.map.sizey);
  inds = sub2ind(size(CMAP.map.data),xis(indGood),yis(indGood));
  CMAP.map.data(inds) = update.cs(indGood);
  
  
  
  
function MapUpdateMsgHandlerH(data,name)
global CMAP ROBOT POSE;
  if isempty(data)
    return
  end
  
  msgSize = length(data);
  fprintf(1,'got horizontal map update\n');
  id = GetIdFromName(name);
  
  update = deserialize(data);
  
  xis = ceil((update.xs - CMAP.xmin) * CMAP.invRes);
  yis = ceil((update.ys - CMAP.ymin) * CMAP.invRes);

  indGood = (xis > 1) & (yis > 1) & (xis < CMAP.map.sizex) & (yis < CMAP.map.sizey);
  inds = sub2ind(size(CMAP.map.data),xis(indGood),yis(indGood));
  CMAP.map.data(inds) = update.cs(indGood);
  
  expandSize = 50;
  xExpand = 0;
  yExpand = 0;

  [xi yi] = Pos2OmapInd(POSE.data.x + [-30  30], POSE.data.y + [-30 30]);
  if (xi(1) < 1), xExpand = -expandSize; end
  if (yi(1) < 1), yExpand = -expandSize; end
  if (xi(2) > CMAP.map.sizex), xExpand = expandSize; end
  if (yi(2) > CMAP.map.sizey), yExpand = expandSize; end

  if (xExpand ~=0 || yExpand ~=0)
    %expand the map
    omapExpand(xExpand,yExpand);
  end
  
  %fprintf(1,'got map update\n');
  
  
function [xi yi] = Pos2OmapInd(x,y)
global CMAP

xi = ceil((x - CMAP.xmin) * CMAP.invRes);
yi = ceil((y - CMAP.ymin) * CMAP.invRes);

