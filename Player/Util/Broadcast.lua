--Steve
--Ashleigh
--This code is used to broadcast each robot's information over network
--Sent string is in lua format (for monitoring)

module(..., package.seeall);


require('Comm')
-- Only send items from shared memory
require('vcm')
require('gcm')
require('wcm')
require('serialization');
require('ImageProc')
require('Config');

-- Initiate Sending Address
Comm.init(Config.dev.ip_wired,111111);
print('Sending to',Config.dev.ip_wired);

-- Add a little delay between packet sending
pktDelay = 500; -- time in us

function sendB()
  -- labelB --
  labelB = vcm.get_image_labelB();
  width = vcm.get_image_width()/8; 
  height = vcm.get_image_height()/8;
  count = vcm.get_image_count();
  
  array = serialization.serialize_array(labelB, width, height, 'uint8', 'labelB', count);
  sendlabelB = {};
  sendlabelB.team = {};
  sendlabelB.team.number = gcm.get_team_number();
  sendlabelB.team.player_id = gcm.get_team_player_id();
  
  for i=1,#array do
    sendlabelB.arr = array[i];
    Comm.send(serialization.serialize(sendlabelB));
  end 
end

function sendA()
  -- labelA --
  labelA = vcm.get_image_labelA();
  width = vcm.get_image_width()/2; 
  height = vcm.get_image_height()/2;
  count = vcm.get_image_count();
  
  array = serialization.serialize_array(labelA, width, height, 'uint8', 'labelA', count);
  sendlabelA = {};
  sendlabelA.team = {};
  sendlabelA.team.number = gcm.get_team_number();
  sendlabelA.team.player_id = gcm.get_team_player_id();
  
  for i=1,#array do
    sendlabelA.arr = array[i];
  	Comm.send(serialization.serialize(sendlabelA));
    -- Need to sleep in order to stop drinking out of firehose
    unix.usleep(pktDelay);
  end
end

function sendImg()
  -- yuyv --
  yuyv = vcm.get_image_yuyv();
  width = vcm.get_image_width()/2; -- number of yuyv packages
  height = vcm.get_image_height();
  count = vcm.get_image_count();
  
  array = serialization.serialize_array(yuyv, width, height, 'int32', 'yuyv', count);
  sendyuyv = {};
  sendyuyv.team = {};
  sendyuyv.team.number = gcm.get_team_number();
  sendyuyv.team.player_id = gcm.get_team_player_id();
  
  for i=1,#array do
    sendyuyv.arr = array[i];
  	Comm.send(serialization.serialize(sendyuyv));
    -- Need to sleep in order to stop drinking out of firehose
    unix.usleep(pktDelay);
  end

end

function update_new(enable)
	if enable == 0 then return; end
        local g = wcm.get_goal()
--        util.ptable( g );
--        util.ptable( g.attack );
	for key,value in pairs(wcm.shared) do 
	--	if (string.find(key,'get')) then
			print('\nget_',key);
                util.ptable( wcm['get_'..key]() )
	--	end

	end	
end

function update(enable)
  if enable==0 then return; end
  
  send = {};

  send.robot = {};
--[[
  local robotpose = wcm.get_robot_pose();
  send.robot.pose = {x=robotpose[1], y=robotpose[2], a=robotpose[3]};
--]]
  send.robot.pose = wcm.get_pose();

  send.ball = {};
  send.ball.detect = vcm.get_ball_detect();
  local ballcentroid = vcm.get_ball_centroid();
  send.ball.centroid = {x=ballcentroid[1], y=ballcentroid[2]};
  send.ball.axisMajor = vcm.get_ball_axisMajor();
  send.ball.axisMinor = vcm.get_ball_axisMinor();
  local ballxy = wcm.get_ball_xy();
  send.ball.x = ballxy[1];
  send.ball.y = ballxy[2];
  send.ball.t = wcm.get_ball_t();

  send.goal = {};
  send.goal.detect = vcm.get_goal_detect();
  send.goal.color = vcm.get_goal_color();
  send.goal.type = vcm.get_goal_type();
  local goalv1 = vcm.get_goal_v1();
  send.goal.v1 = {x=goalv1[1], y=goalv1[2], z=goalv1[3], scale=goalv1[4]};
  local goalv2 = vcm.get_goal_v2();
  send.goal.v2 = {x=goalv2[1], y=goalv2[2], z=goalv2[3], scale=goalv2[4]};
  local bb1 = vcm.get_goal_postBoundingBox1();
  send.goal.postBoundingBox1 = {x1=bb1[1], x2=bb1[2], y1=bb1[3], y2=bb1[4]};
  local bb2 = vcm.get_goal_postBoundingBox2();
  send.goal.postBoundingBox2 = {x1=bb2[1], x2=bb2[2], y1=bb2[3], y2=bb2[4]};

  send.time = unix.time();

  send.team = {};
  send.team.number = gcm.get_team_number();
  send.team.player_id = gcm.get_team_player_id();
  send.team.color = gcm.get_team_color();
  send.team.role = gcm.get_team_role();
  send.team.attackBearing = wcm.get_attack_bearing();
  send.team.penalty = gcm.get_game_penalty( gcm.get_team_player_id() );

  Comm.send(serialization.serialize(send));
  
end

function update_img( enable, imagecount )
  local division = 4; -- for image sending part by part
  if(enable==2) then
    sendB();
    sendImg(); -- half of sub image
    sendA();
--    sendImgSub(2);
  elseif(enable==3) then
	if (Config.platform.name ~= "Nao") then
--	    sendImgSub();
-- 	  sendAsub();
		end
  end
end
