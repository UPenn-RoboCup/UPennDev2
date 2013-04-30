function receiveIncMap(host)
global POSE USER_INPUT VIS

if nargin < 1
  host = 'localhost';
end

SetMagicPaths;

%ipcInit('192.168.10.100');
ipcInit(host)
poseInit;
cmapInit;
omapInit;
emapInit;

POSE.cntr =1;
USER_INPUT.freshClick =0;
%id of the robot that maps should be received from
setenv('ROBOT_ID','1');


ipcReceiveSetFcn(GetMsgName('Pose'), @PoseMsgHander);
ipcReceiveSetFcn(GetMsgName('MapIncUpdate'), @MapUpdateMsgHandler);

%set the message queue lengths so that messages dont get built up on
%central
ipcAPI('set_msg_queue_length',GetMsgName('Pose'),1);
ipcAPI('set_msg_queue_length',GetMsgName('MapIncUpdate'),10);

trajMsgName = GetMsgName('Traj');
ipcAPIDefine(trajMsgName);

figure(1), clf(gcf);
drawnow;
set(gcf,'WindowButtonUpFcn',@mouseClickCallback);



%vis stuff
VIS.mapMsgName = 'Robot1/CostMap2D_map2d';
mapMsgFormat = VisMap2DSerializer('getFormat');

VIS.updateRectMsgName   = 'Robot1/CostMap2D_map2dUpdateRect';
updateRectMsgFormat = VisMap2DUpdateRectSerializer('getFormat'); 

VIS.updatePointsMsgName   = 'Robot1/CostMap2D_map2dUpdatePoints';
updatePointsMsgFormat = VisMap2DUpdatePointsSerializer('getFormat'); 

ipcAPIConnect;
ipcAPIDefine(VIS.mapMsgName,mapMsgFormat);
ipcAPIDefine(VIS.updateRectMsgName,updateRectMsgFormat);
ipcAPIDefine(VIS.updatePointsMsgName,updatePointsMsgFormat);




%%%%%






while(1)
  ipcReceiveMessages;
  
  if (USER_INPUT.freshClick)
    fprintf(1,'got user input %f %f\n',USER_INPUT.x, USER_INPUT.y);
    USER_INPUT.freshClick = 0;
    
    traj.size = 1;
    traj.waypoints(1).y = USER_INPUT.x;    %switch the order here
    traj.waypoints(1).x = USER_INPUT.y;
    goal.x = USER_INPUT.x;
    goal.y = USER_INPUT.y;
    ipcAPIPublish(trajMsgName,serialize(traj));
    fprintf(1,'published traj\n');
  end
end

function PoseMsgHander(data,name)
global POSE MAP_FIGURE
  if isempty(data)
    return;
  end
  
  POSE.data = MagicPoseSerializer('deserialize',data);
  
  if isempty(MAP_FIGURE)
    return
  end
  
  if ~isfield(POSE,'hPose')
    hold on;
    POSE.hPose = plot(POSE.data.x,POSE.data.y,'r*');
    hold off;
  else
    set(POSE.hPose,'xdata',POSE.data.x,'ydata',POSE.data.y);
  end
  
  POSE.cntr = POSE.cntr +1;
  if (mod(POSE.cntr,10) == 0)
    drawnow;
  end
  
  %fprintf(1,'got pose update\n');

function MapUpdateMsgHandler(data,name)
global CMAP MAP_FIGURE POSE VIS
  if isempty(data)
    return
  end
  
  msgSize = length(data);
  fprintf(1,'got map update of size %d\n',msgSize);
  update = deserialize(data);
  %plot(update.cs); drawnow;
  xis = ceil((update.xs - CMAP.xmin) * CMAP.invRes);
  yis = ceil((update.ys - CMAP.ymin) * CMAP.invRes);
  
  indGood = (xis > 1) & (yis > 1) & (xis < CMAP.map.sizex) & (yis < CMAP.map.sizey);
  inds = sub2ind(size(CMAP.map.data),xis(indGood),yis(indGood));
  
  CMAP.map.data(inds) = update.cs(indGood);
  
  if isempty(MAP_FIGURE)
    hold on;
    MAP_FIGURE.hMap = imagesc(CMAP.map.data'); %transpose to make x horizontal
    set(MAP_FIGURE.hMap,'xdata',[CMAP.xmin CMAP.xmax], ...
             'ydata',[CMAP.ymin CMAP.ymax]);
    colormap gray;
    hold off;
  else
    set(MAP_FIGURE.hMap,'xdata',[CMAP.xmin CMAP.xmax], ...
                        'ydata',[CMAP.ymin CMAP.ymax], ...
                        'cdata',CMAP.map.data');
  end
  drawnow;
  
  map.xmin      = CMAP.xmin;
  map.ymin      = CMAP.ymin;
  map.res       = CMAP.res;
  map.map.sizex = CMAP.map.sizex;
  map.map.sizey = CMAP.map.sizey;
  map.map.data  = uint8(CMAP.map.data + 127);
  content = VisMap2DSerializer('serialize',map);
  ipcAPIPublishVC(VIS.mapMsgName,content);
  
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
