function h = shm_robot(teamNumber, playerID)
% function create the same struct as the team message from
% shared memory. for local debugging use

h.teamNumber = teamNumber;
h.playerID = playerID;
h.user = getenv('USER');

% create shm wrappers
h.gcmTeam  = shm(sprintf('gcmTeam%d%d%s',  h.teamNumber, h.playerID, h.user));
h.wcmRobot = shm(sprintf('wcmRobot%d%d%s', h.teamNumber, h.playerID, h.user));
h.wcmBall  = shm(sprintf('wcmBall%d%d%s',  h.teamNumber, h.playerID, h.user));
h.wcmGoal  = shm(sprintf('wcmGoal%d%d%s',  h.teamNumber, h.playerID, h.user));
h.vcmImage = shm(sprintf('vcmImage%d%d%s', h.teamNumber, h.playerID, h.user));
h.vcmBall  = shm(sprintf('vcmBall%d%d%s',  h.teamNumber, h.playerID, h.user));
h.vcmGoal  = shm(sprintf('vcmGoal%d%d%s',  h.teamNumber, h.playerID, h.user));
% shm wrappers for freespace, occumap, boundary
h.wcmOccmap = shm(sprintf('wcmOccmap%d%d%s', h.teamNumber, h.playerID, h.user));
h.vcmFreespace = shm(sprintf('vcmFreespace%d%d%s', h.teamNumber, h.playerID, h.user));
h.vcmBoundary = shm(sprintf('vcmBoundary%d%d%s', h.teamNumber, h.playerID, h.user));

% set function pointers
h.update = @update;
h.get_team_struct = @get_team_struct;
h.get_monitor_struct = @get_monitor_struct;
h.get_yuyv = @get_yuyv;
h.get_rgb = @get_rgb;
h.get_labelA = @get_labelA;
h.get_labelB = @get_labelB;

  function update()
      % do nothing
  end

  function r = get_team_struct()
    % returns the robot struct (in the same form as the team messages)
    r = [];
    disp('get team struct');
    try
        r.teamNumber = h.gcmTeam.get_number();
        r.teamColor = h.gcmTeam.get_color();
        r.id = h.gcmTeam.get_player_id();
        r.role = h.gcmTeam.get_role();
        
        pose = h.wcmRobot.get_pose();
        r.pose = struct('x', pose(1), 'y', pose(2), 'a', pose(3));
        
        ballxy = h.wcmBall.get_xy();
        ballt = h.wcmBall.get_t();
        ballvel = h.wcmBall.get_velocity();
        r.ball = struct('x', ballxy(1), 'y', ballxy(2), 't', ballt, ...
            'vx', ballvel(1), 'vy', ballvel(2) );
        
        % TODO: implement penalty and time
        r.penalty = 0;
        r.attackBearing = h.wcmGoal.get_attack_bearing();
        r.time = 0;
        r.tReceive = 0;
        
    catch
    end
  end

  function r = get_monitor_struct()
    % returns the monitor struct (in the same form as the monitor messages)
    r = [];
    try
      r.team = struct(...
          'number', h.gcmTeam.get_number(),...
          'color', h.gcmTeam.get_color(),...
          'player_id', h.gcmTeam.get_player_id(),...
          'role', h.gcmTeam.get_role()...
          );
   
      pose = h.wcmRobot.get_pose();
      r.robot = {};
      r.robot.pose = struct('x', pose(1), 'y', pose(2), 'a', pose(3));
          
      ballxy = h.wcmBall.get_xy();
      ballt = h.wcmBall.get_t();
      ball = {};
      ball.detect = h.vcmBall.get_detect();
      ball.centroid = {};
      centroid = h.vcmBall.get_centroid();
      ball.centroid.x = centroid(1);
      ball.centroid.y = centroid(2);
      ball.axisMajor = h.vcmBall.get_axisMajor();
      r.ball = struct('x', ballxy(1), 'y', ballxy(2), 't', ballt, ...
          'centroid', ball.centroid, 'axisMajor', ball.axisMajor, ...
          'detect', ball.detect);
      r.goal = {};
      r.goal.detect = h.vcmGoal.get_detect();
      r.goal.type = h.vcmGoal.get_type();
      r.goal.color = h.vcmGoal.get_color();
          
      % Add the goal positions
      goalv1 = h.vcmGoal.get_v1();
      r.goal.v1 = struct('x',goalv1(1), 'y',goalv1(2), 'z',goalv1(3), 'scale',goalv1(4));
      goalv2 = h.vcmGoal.get_v2();
      r.goal.v2 = struct('x',goalv2(1), 'y',goalv2(2), 'z',goalv2(3), 'scale',goalv2(4));
          
      % Add the goal bounding boxes
      gbb1 = h.vcmGoal.get_postBoundingBox1();
      r.goal.postBoundingBox1 = struct('x1',gbb1(1), 'x2',gbb1(2), 'y1',gbb1(3), 'y2',gbb1(4));
      gbb2 = h.vcmGoal.get_postBoundingBox2();
      r.goal.postBoundingBox2 = struct('x1',gbb2(1), 'x2',gbb2(2), 'y1',gbb2(3), 'y2',gbb2(4));
      
      % Add freespace boundary
      r.free = {};
      freeCol = h.vcmFreespace.get_nCol();
%		   freeValueA = h.vcmFreespace.get_pboundA();
      freeValueB = h.vcmFreespace.get_pboundB();
      labelAm = h.vcmImage.get_width()/2;
      labelBm = labelAm/4;
%      r.free = struct('Ax',freeValueA(1:labelAm),
%                      'Ay',freeValueA(labelAm+1:2*labelAm),
      r.free = struct('Bx',freeValueB(1:labelBm),...
                      'By',freeValueB(labelBm+1:2*labelBm),...
                      'nCol',freeCol,...
                      'detect',h.vcmFreespace.get_detect());
      % Add visible boundary        

      
      r.bd = {};
      bdTop = h.vcmBoundary.get_top();
	    bdBtm = h.vcmBoundary.get_bottom();
      bdCol = size(bdTop,2)/2;
      r.bd = struct('detect',h.vcmBoundary.get_detect(),...
                    'nCol',bdCol,...
                    'topy',bdTop(1,1:bdCol),...
                    'topx',-bdTop(1,bdCol+1:2*bdCol),...
                    'btmy',bdBtm(1,1:bdCol),...
                    'btmx',-bdBtm(1,bdCol+1:2*bdCol));
      % Add occupancy map

      r.occ = {};
      div = size(h.wcmOccmap.get_r(),2);
      interval = 2*pi/div;
      r.occ = struct('div',div,'interval',interval,...
                     'halfInter',interval/2,...
                     'r',h.wcmOccmap.get_r(),...
                     'theta',zeros(div*4,1),...
                     'rho',zeros(div*4,1),...
                     'x',zeros(div*4,1),...
                     'y',zeros(div*4,1));
      % add horizon line
      r.horizon = {};
      labelAm = h.vcmImage.get_width()/2;
      labelBm = labelAm/4;
	    horizonDir = h.vcmImage.get_horizonDir();
      horizonA = h.vcmImage.get_horizonA();
      horizonB = h.vcmImage.get_horizonB();
      horizonAx = 1 : labelAm;
      horizonBx = 1 : labelBm;
      horizonAy = (horizonAx - horizonAx(end)/2) .* tan(horizonDir) + horizonA;
      horizonBy = (horizonBx - horizonBx(end)/2) .* tan(horizonDir) + horizonB;
      r.horizon = struct('hYA',horizonAy,...
                         'hYB',horizonBy,...
                         'hXA',horizonAx,...
                         'hXB',horizonBx);
    catch
    end 
  end

  function yuyv = get_yuyv()
      % returns the raw YUYV image
%         width = h.vcmImage.get_width();
%         height = h.vcmImage.get_height();
    width = h.vcmImage.get_width()/2;
    height = h.vcmImage.get_height();
    rawData = h.vcmImage.get_yuyv();
    yuyv = raw2yuyv(rawData, width, height); %for Nao, double for OP
  end

  function rgb = get_rgb()
    % returns the raw RGB image (not full size)
    yuyv = h.get_yuyv();
    rgb = yuyv2rgb(yuyv);
  end

  function labelA = get_labelA()
    % returns the labeled image
    width = h.vcmImage.get_width()/2;
    height = h.vcmImage.get_height()/2;
    rawData = h.vcmImage.get_labelA();
    labelA = raw2label(rawData, width, height)';
  end

  function labelB = get_labelB()
    % returns the bit-ored labeled image
    width = h.vcmImage.get_width()/4/2;
    height = h.vcmImage.get_height()/4/2;
    rawData = h.vcmImage.get_labelB();
    labelB = raw2label(rawData, width, height)';
  end
end

