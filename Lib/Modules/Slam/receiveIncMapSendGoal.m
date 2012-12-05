function receiveIncMapSendGoal(id,addr)

global POSE USER_INPUT

SetMagicPaths;

if nargin < 1
  id = 0;
end

if nargin < 2
  addr = 'localhost';
end

%ipcInit('192.168.10.102');
ipcInit(addr);
poseInit;
initMapProps;
omapInit;
emapInit;

POSE.cntr =1;
USER_INPUT.freshClick =0;
%id of the robot that maps should be received from
setenv('ROBOT_ID',sprintf('%d',id));

ipcReceiveSetFcn(GetMsgName('Pose'), @PoseMsgHander);
ipcReceiveSetFcn(GetMsgName('IncMapUpdateH'), @MapUpdateMsgHandler);

%set the message queue lengths so that messages dont get built up on
%central
ipcAPI('set_msg_queue_length',GetMsgName('Pose'),1);
ipcAPI('set_msg_queue_length',GetMsgName('IncMapUpdateH'),10);

trajMsgName = GetMsgName('Goal');
ipcAPIDefine(trajMsgName);

figure(1), clf(gcf);
drawnow;
set(gcf,'WindowButtonUpFcn',@mouseClickCallback);

while(1)
  ipcReceiveMessages;
  
  if (USER_INPUT.freshClick)
    fprintf(1,'got user input %f %f\n',USER_INPUT.x, USER_INPUT.y);
    USER_INPUT.freshClick = 0;
    
    traj.size = 1;
    traj.waypoints(1).y = USER_INPUT.x;
    traj.waypoints(1).x = USER_INPUT.y;
    ipcAPIPublish(trajMsgName,serialize(traj));
    fprintf(1,'published traj\n');
  end
end

function PoseMsgHander(msg,name)
global POSE MAP_FIGURE
  if isempty(msg)
    return;
  end
  
  POSE.data = MagicPoseSerializer('deserialize',msg);
  
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

function MapUpdateMsgHandler(msg,name)
global OMAP MAP_FIGURE POSE
  if isempty(msg)
    return
  end
  
  msgSize = length(msg);
  fprintf(1,'got map update of size %d\n',msgSize);
  update = deserialize(msg);
  xis = ceil((update.xs - OMAP.xmin) * OMAP.invRes);
  yis = ceil((update.ys - OMAP.ymin) * OMAP.invRes);
  
  indGood = (xis > 1) & (yis > 1) & (xis < OMAP.map.sizex) & (yis < OMAP.map.sizey);
  inds = sub2ind(size(OMAP.map.data),xis(indGood),yis(indGood));
  
  OMAP.map.data(inds) = update.cs(indGood);
  
  if isempty(MAP_FIGURE)
    hold on;
    MAP_FIGURE.hMap = image(100-OMAP.map.data'); %transpose to make x horizontal
    set(MAP_FIGURE.hMap,'xdata',[OMAP.xmin OMAP.xmax], ...
             'ydata',[OMAP.ymin OMAP.ymax]);
    colormap gray;
    hold off;
  else
    set(MAP_FIGURE.hMap,'xdata',[OMAP.xmin OMAP.xmax], ...
                        'ydata',[OMAP.ymin OMAP.ymax], ...
                        'cdata',100-OMAP.map.data');
  end
  drawnow;
  
  expandSize = 50;
  xExpand = 0;
  yExpand = 0;

  [xi yi] = Pos2OmapInd(POSE.data.x + [-30  30], POSE.data.y + [-30 30]);
  if (xi(1) < 1), xExpand = -expandSize; end
  if (yi(1) < 1), yExpand = -expandSize; end
  if (xi(2) > OMAP.map.sizex), xExpand = expandSize; end
  if (yi(2) > OMAP.map.sizey), yExpand = expandSize; end

  if (xExpand ~=0 || yExpand ~=0)
    %expand the map
    omapExpand(xExpand,yExpand);
  end
  
  %fprintf(1,'got map update\n');
  
  
function [xi yi] = Pos2OmapInd(x,y)
global OMAP

xi = ceil((x - OMAP.xmin) * OMAP.invRes);
yi = ceil((y - OMAP.ymin) * OMAP.invRes);
