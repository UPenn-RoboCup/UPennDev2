module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');
require('Body');
require('vcm');

-- Dependency
require('detectBall');
require('detectGoal');
require('detectLine');
require('detectCorner');
require('detectLandmarks'); -- for NSL
require('detectSpot');
require('detectFreespace');
require('detectBoundary');
--[[
require('detectObstacles');
require('detectEyes');
require('detectStretcher');
--]]


--for quick test
require('detectRobot');


-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

use_point_goal=Config.vision.use_point_goal;

enableLine = Config.vision.enable_line_detection;
enableSpot = Config.vision.enable_spot_detection;
enableMidfieldLandmark = Config.vision.enable_midfield_landmark_detection;
enableFreespace = Config.vision.enable_freespace_detection or 0;
enableBoundary = Config.vision.enable_visible_boundary or 0;
enableRobot = Config.vision.enable_robot_detection or 0;


function entry()
  -- Initiate Detection
  ball = {};
  ball.detect = 0;

  ballYellow={};
  ballYellow.detect=0;
	
  ballCyan={};
  ballCyan.detect=0;

  goalYellow = {};
  goalYellow.detect = 0;

  goalCyan = {};
  goalCyan.detect = 0;

  landmarkYellow = {};
  landmarkYellow.detect = 0;

  landmarkCyan = {};
  landmarkCyan.detect = 0;

  line = {};
  line.detect = 0;

  corner = {};
  corner.detect = 0;
  
  spot = {};
  spot.detect = 0;

  obstacle={};
  obstacle.detect=0;

  freespace={};
  freespace.detect=0;

  boundary={};
  boundary.detect=0;


end

function update()

  if( Config.gametype == "stretcher" ) then
    ball = detectEyes.detect(colorOrange);
    return;
  end

  -- ball detector
  ball = detectBall.detect(colorOrange);

  -- goal detector
  if use_point_goal == 1 then
    ballYellow = detectBall.detect(colorYellow);
    ballCyan = detectBall.detect(colorCyan);
  else
--SJ: we need to detect both colored goalposts (due to landmarks)
--TODO: single-colored goalpost
    goalYellow.detect=0;
    goalCyan.detect=0;
    goalYellow = detectGoal.detect(colorYellow,colorCyan);
    goalCyan = detectGoal.detect(colorCyan,colorYellow);
--[[
    --if (colorCount[colorYellow] > colorCount[colorCyan]) then
    if (Vision.colorCount[colorYellow] > yellowGoalCountThres) then
      goalYellow = detectGoal.detect(colorYellow,colorCyan);
      goalCyan.detect = 0;
    else
      goalCyan = detectGoal.detect(colorCyan,colorYellow);
      goalYellow.detect = 0;
    end
--]]
  end

  -- line detection
  if enableLine == 1 then
    line = detectLine.detect();
    corner=detectCorner.detect(line);
  end

  -- spot detection
  if enableSpot == 1 then
--    spot = detectSpot.detect();
  end

  -- midfield landmark detection
  landmarkCyan = 0;
  landmarkYellow = 0;
  if enableMidfieldLandmark == 1 then
    landmarkCyan = detectLandmarks.detect(colorCyan,colorYellow);
    landmarkYellow = detectLandmarks.detect(colorYellow,colorCyan);
  end

  -- freespace detection
  if enableFreespace == 1 then
    freespace = detectFreespace.detect(colorField);
  end

  -- visible boundary detection
  if enableBoundary == 1 then
    boundary = detectBoundary.detect();
  end

  if enableRobot ==1 then
    --for quick testing
    detectRobot.detect();
  end

end

function update_shm()
  vcm.set_ball_detect(ball.detect);
  if (ball.detect == 1) then
    vcm.set_ball_centroid(ball.propsA.centroid);
    vcm.set_ball_axisMajor(ball.propsA.axisMajor);
    vcm.set_ball_axisMinor(ball.propsA.axisMinor);
    vcm.set_ball_v(ball.v);
    vcm.set_ball_r(ball.r);
    vcm.set_ball_dr(ball.dr);
    vcm.set_ball_da(ball.da);
  end

  vcm.set_goal_detect(math.max(goalCyan.detect, goalYellow.detect));
  if (goalCyan.detect == 1) then
    vcm.set_goal_color(colorCyan);
    vcm.set_goal_type(goalCyan.type);
    vcm.set_goal_v1(goalCyan.v[1]);
    vcm.set_goal_v2(goalCyan.v[2]);
  elseif (goalYellow.detect == 1) then
    vcm.set_goal_color(colorYellow);
    vcm.set_goal_type(goalYellow.type);
    vcm.set_goal_v1(goalYellow.v[1]);
    vcm.set_goal_v2(goalYellow.v[2]);
  end

  -- midfield landmark detection
  vcm.set_landmark_detect(0);
  if enableMidfieldLandmark == 1 then
    if landmarkYellow.detect==1 then
       vcm.set_landmark_detect(1);
       vcm.set_landmark_color(colorYellow);
       vcm.set_landmark_v(landmarkYellow.v);
    elseif landmarkCyan.detect==1 then
       vcm.set_landmark_detect(1);
       vcm.set_landmark_color(colorCyan);
       vcm.set_landmark_v(landmarkCyan.v);
    end
  end

  vcm.set_line_detect(line.detect);
  if (line.detect == 1) then
    vcm.set_line_nLines(line.nLines);
    local v1x=vector.zeros(6);
    local v1y=vector.zeros(6);
    local v2x=vector.zeros(6);
    local v2y=vector.zeros(6);
    local endpoint11=vector.zeros(6);
    local endpoint12=vector.zeros(6);
    local endpoint21=vector.zeros(6);
    local endpoint22=vector.zeros(6);

    max_length=0;
    max_index=1;
    for i=1,line.nLines do 
      v1x[i]=line.v[i][1][1];
      v1y[i]=line.v[i][1][2];
      v2x[i]=line.v[i][2][1];
      v2y[i]=line.v[i][2][2];
      --x0 x1 y0 y1
      endpoint11[i]=line.endpoint[i][1];
      endpoint12[i]=line.endpoint[i][3];
      endpoint21[i]=line.endpoint[i][2];
      endpoint22[i]=line.endpoint[i][4];
      if max_length<line.length[i] then
        max_length=line.length[i];
	max_index=i;
      end
    end

    vcm.set_line_v1x(v1x);
    vcm.set_line_v1y(v1y);
    vcm.set_line_v2x(v2x);
    vcm.set_line_v2y(v2y);
    vcm.set_line_endpoint11(endpoint11);
    vcm.set_line_endpoint12(endpoint12);
    vcm.set_line_endpoint21(endpoint21);
    vcm.set_line_endpoint22(endpoint22);

    vcm.set_line_v({(v1x[max_index]+v2x[max_index])/2,
	 	   (v1y[max_index]+v2y[max_index])/2,0,0});
    vcm.set_line_angle(line.angle[max_index]);

  end
  vcm.set_corner_detect(corner.detect);
  if (corner.detect == 1) then
    vcm.set_corner_type(corner.type)
    vcm.set_corner_vc0(corner.vc0)
    vcm.set_corner_v10(corner.v10)
    vcm.set_corner_v20(corner.v20)
    vcm.set_corner_v(corner.v)
    vcm.set_corner_v1(corner.v1)
    vcm.set_corner_v2(corner.v2)
  end

  --vcm.set_spot_detect(spot.detect);
  if (spot.detect == 1) then
  end

  vcm.set_freespace_detect(freespace.detect);
  if (freespace.detect == 1) then
	vcm.set_freespace_block(freespace.block);
    vcm.set_freespace_nCol(freespace.nCol);
    vcm.set_freespace_nRow(freespace.nRow);
--    vcm.set_freespace_vboundA(freespace.vboundA);
--    vcm.set_freespace_pboundA(freespace.pboundA);
--    vcm.set_freespace_tboundA(freespace.tboundA);
    vcm.set_freespace_vboundB(freespace.vboundB);
    vcm.set_freespace_pboundB(freespace.pboundB);
    vcm.set_freespace_tboundB(freespace.tboundB);
  end

  vcm.set_boundary_detect(boundary.detect);
  if (boundary.detect == 1) then
	if (freespace.detect == 1) then
		vcm.set_boundary_top(freespace.vboundB);
	else
		vcm.set_boundary_top(boundary.top);
	end
	vcm.set_boundary_bottom(boundary.bottom);
  end

end

function exit()
end
