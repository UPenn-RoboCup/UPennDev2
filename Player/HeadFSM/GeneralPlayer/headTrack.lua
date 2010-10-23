module(..., package.seeall);

require('Body')
require('HeadTransform')
require('Config')
require('wcm')

t0 = 0;

minDist = Config.fsm.headTrack.minDist;
fixTh = Config.fsm.headTrack.fixTh;
trackZ = Config.vision.ball_diameter; 
timeout = Config.fsm.headTrack.timeout;
tLost = Config.fsm.headTrack.tLost;

min_eta_look = Config.min_eta_look or 2.0;


goalie_dive = Config.goalie_dive or 0;
goalie_type = Config.fsm.goalie_type;


function entry()
  print("Head SM:".._NAME.." entry");

  t0 = Body.get_time();
  vcm.set_camera_command(-1); --switch camera

end

function update()

  role = gcm.get_team_role();
  --Force attacker for demo code
  if Config.fsm.playMode==1 then role=1; end
  if role==0 and goalie_type>2 then --Escape if diving goalie
    return "goalie";
  end

  local t = Body.get_time();

  -- update head position based on ball location
  ball = wcm.get_ball();
  ballR = math.sqrt (ball.x^2 + ball.y^2);

  local yaw,pitch;
  --top:0 bottom: 1
  
  if Config.platform.name== 'WebotsNao' or
     Config.platform.name== 'NaoV4' then

    --Bottom camera check
    yaw, pitchBottom =
	HeadTransform.ikineCam(ball.x, ball.y, trackZ, 1);

    --Do we need to look down?
    if pitchBottom > 10*math.pi/180 then
      pitch = pitchBottom - 10*math.pi/180;
    else
      pitch = 0;
    end

--    print(pitchBottom*180/math.pi, pitch*180/math.pi)

  else --OP: look at the ball
    yaw, pitch =
	HeadTransform.ikineCam(ball.x, ball.y, trackZ, 0);
  end
  

  -- Fix head yaw while approaching (to reduce position error)
  if ball.x<fixTh[1] and math.abs(ball.y) < fixTh[2] then
     yaw=0.0; 
  end
  Body.set_head_command({yaw, pitch});

  if (t - ball.t > tLost) then
    print('Ball lost!');
    return "lost";
  end

  eta = wcm.get_team_my_eta();
  if eta<min_eta_look and eta>0 then
    return;
  end

  if (t - t0 > timeout) then
     if role==0 then
       return "sweep"; --Goalie, sweep to localize
     else
       return "timeout";  --Player, look up to see goalpost
     end
  end
end

function exit()
end
