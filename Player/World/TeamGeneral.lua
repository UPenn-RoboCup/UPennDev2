module(..., package.seeall);

require('Config');
require('Body');
require('Comm');
require('Speak');
require('vector');
require('util')
require('serialization');

require('wcm');
require('gcm');

Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);
print('Receiving Team Message From',Config.dev.ip_wireless);
playerID = gcm.get_team_player_id();

msgTimeout = Config.team.msgTimeout;
nonAttackerPenalty = Config.team.nonAttackerPenalty;
nonDefenderPenalty = Config.team.nonDefenderPenalty;
fallDownPenalty = Config.team.fallDownPenalty;
ballLostPenalty = Config.team.ballLostPenalty;

walkSpeed = Config.team.walkSpeed;
turnSpeed = Config.team.turnSpeed;

flip_correction = Config.team.flip_correction or 0;
goalie_ball={0,0,0};

--Player ID: 1 to 5
--to prevent confusion, now we use these definitions
ROLE_GOALIE = 0;
ROLE_ATTACKER = 1;
ROLE_DEFENDER = 2;
ROLE_SUPPORTER = 3;
ROLE_DEFENDER2 = 4;
ROLE_RESERVE_PLAYER = 5;
ROLE_RESERVE_GOALIE = 6;

count = 0;

state = {};
state.robotName = Config.game.robotName;
state.teamNumber = gcm.get_team_number();
state.id = playerID;
state.teamColor = gcm.get_team_color();
state.time = Body.get_time();
state.role = -1;
state.pose = {x=0, y=0, a=0};
state.ball = {t=0, x=1, y=0, vx=0, vy=0, p = 0};
state.attackBearing = 0.0;--Why do we need this?
state.penalty = 0;
state.tReceive = Body.get_time();
state.battery_level = wcm.get_robot_battery_level();
state.fall=0;

--Added key vision infos
state.goal=0;  --0 for non-detect, 1 for unknown, 2/3 for L/R, 4 for both
state.goalv1={0,0};
state.goalv2={0,0};
state.goalB1={0,0,0,0,0};--Centroid X Centroid Y Orientation Axis1 Axis2
state.goalB2={0,0,0,0,0};
state.landmark=0; --0 for non-detect, 1 for yellow, 2 for cyan
state.landmarkv={0,0};
state.corner=0; --0 for non-detect, 1 for L, 2 for T
state.cornerv={0,0};

--Game state info
state.gc_latency=0;
state.tm_latency=0;

--Body state 
state.bodyState = gcm.get_fsm_body_state();

states = {};
states[playerID] = state;

--We maintain pose of all robots 
--For obstacle avoidance
poses={};
player_roles=vector.zeros(10);
t_poses=vector.zeros(10);
tLastMessage = 0;


function recv_msgs()
  while (Comm.size() > 0) do 
    msg=Comm.receive();
    --Ball GPS Info hadling
    if msg and #msg==14 then --Ball position message
      ball_gpsx=(tonumber(string.sub(msg,2,6))-5)*2;
      ball_gpsy=(tonumber(string.sub(msg,8,12))-5)*2;
      wcm.set_robot_gps_ball({ball_gpsx,ball_gpsy,0});
    else --Regular team message
      t = serialization.deserialize(msg);
      --    t = unpack_msg(msg);
      if t and (t.teamNumber) and (t.id) then
        tLastMessage = Body.get_time();
        --Messages from upenn code
        --Keep all pose data for collison avoidance 
        if t.teamNumber ~= state.teamNumber then
          poses[t.id+5]=t.pose;
          player_roles[t.id+5]=t.role;
          t_poses[t.id+5]=Body.get_time();
        elseif t.id ~=playerID then
          poses[t.id]=t.pose;
          player_roles[t.id]=t.role;
          t_poses[t.id]=Body.get_time();
        end
        --Is the message from our team?
        if (t.teamNumber == state.teamNumber) and 
          (t.id ~= playerID) then
          t.tReceive = Body.get_time();
          t.labelB = {}; --Kill labelB information
          states[t.id] = t;
        end
      end
    end
  end
end

function update_obstacle()
  --Update local obstacle information based on other robots' localization info
  local t = Body.get_time();
  local t_timeout = 2.0;
  local closest_pose={};
  local closest_dist =100;
  local closest_index = 0;
  local closest_role = 0;
  pose = wcm.get_pose();
  avoid_other_team = Config.avoid_other_team or 0;
  if avoid_other_team>0 then num_teammates = 10;end
  obstacle_count = 0;
  obstacle_x=vector.zeros(10);
  obstacle_y=vector.zeros(10);
  obstacle_dist=vector.zeros(10);
  obstacle_role=vector.zeros(10);
  for i=1,10 do
    if t_poses[i]~=0 and 
      t-t_poses[i]<t_timeout and
      player_roles[i]<ROLE_RESERVE_PLAYER then
      obstacle_count = obstacle_count+1;
      local obstacle_local = util.pose_relative({poses[i].x,poses[i].y,0},{pose.x,pose.y,pose.a}); 
      dist = math.sqrt(obstacle_local[1]^2+obstacle_local[2]^2);
      obstacle_x[obstacle_count]=obstacle_local[1];
      obstacle_y[obstacle_count]=obstacle_local[2];
      obstacle_dist[obstacle_count]=dist;
      if i<6 then --Same team
        obstacle_role[obstacle_count] = player_roles[i]; --0,1,2,3,4
      else --Opponent team
        obstacle_role[obstacle_count] = player_roles[i]+5; --5,6,7,8,9
      end
    end
  end
  wcm.set_obstacle_num(obstacle_count);
  wcm.set_obstacle_x(obstacle_x);
  wcm.set_obstacle_y(obstacle_y);
  wcm.set_obstacle_dist(obstacle_dist);
  wcm.set_obstacle_role(obstacle_role);
  --print("Closest index dist", closest_index, closest_dist);
end

function entry()
end


function update()
  --print("====PLAYERID:",playerID);
  count = count + 1;
  state.time = Body.get_time();
  state.teamNumber = gcm.get_team_number();
  state.teamColor = gcm.get_team_color();
  state.pose = wcm.get_pose();
  state.ball = wcm.get_ball();
  state.role = role;
  state.attackBearing = wcm.get_attack_bearing();
  state.battery_level = wcm.get_robot_battery_level();
  state.fall=wcm.get_robot_is_fall_down();
  state.bodyState = gcm.get_fsm_body_state();

  if gcm.in_penalty() then  state.penalty = 1;
  else  state.penalty = 0;
  end

  --Set gamecontroller latency info
  state.gc_latency=gcm.get_game_gc_latency();
  state.tm_latency=Body.get_time()-tLastMessage;

  pack_vision_info(); --Vision info
  pack_labelB(); --labelB info

  --Now pack state name too
  state.body_state = gcm.get_fsm_body_state();

  if (math.mod(count, 1) == 0) then --TODO: How often can we send team message?
    msg=serialization.serialize(state);
    Comm.send(msg, #msg);
    state.tReceive = Body.get_time();
    states[playerID] = state;
  end

  -- receive new messages every frame
  recv_msgs();

  -- eta and defend distance calculation:
  eta = {};
  ddefend = {};
  roles = {};
  t = Body.get_time();
  for id = 1,5 do 
    if not states[id] or not states[id].ball.x then  -- no info from player, ignore him
      eta[id] = math.huge;
      ddefend[id] = math.huge;
      roles[id]=ROLE_RESERVE_PLAYER; 
    else    -- Estimated Time of Arrival to ball (in sec)
--[[
--Old ETA calculation:
      eta[id] = rBall/0.10 +  4*math.max(tBall-1.0,0)+
      math.abs(states[id].attackBearing)/3.0; --1 sec to turn 180 deg
--]]

      --New ETA calculation considering turning, ball uncertainty
      --walkSpeed: seconds needed to walk 1m
      --turnSpeed: seconds needed to turn 360 degrees
      --TODO: Consider sidekick

      rBall = math.sqrt(states[id].ball.x^2 + states[id].ball.y^2);
      tBall = states[id].time - states[id].ball.t;
      eta[id] = rBall/walkSpeed + --Walking time
        math.abs(states[id].attackBearing)/(2*math.pi)*turnSpeed+ --Turning 
        ballLostPenalty * math.max(tBall-1.0,0);  --Ball uncertainty

      roles[id]=states[id].role;
      dgoalPosition = vector.new(wcm.get_goal_defend());-- distance to our goal

      ddefend[id] = 
	math.sqrt((states[id].pose.x - dgoalPosition[1])^2 +
		 (states[id].pose.y - dgoalPosition[2])^2);

      if (states[id].role ~= ROLE_ATTACKER ) then       -- Non attacker penalty:
        eta[id] = eta[id] + nonAttackerPenalty/walkSpeed;
      end

      -- Non defender penalty:
      if (states[id].role ~= ROLE_DEFENDER and states[id].role~=ROLE_DEFENDER2) then 
        ddefend[id] = ddefend[id] + 0.3;
      end

      if (states[id].fall==1) then  --Fallen robot penalty
        eta[id] = eta[id] + fallDownPenalty;
      end

      --Ignore goalie, reserver, penalized player
      if (states[id].penalty > 0) or 
        (t - states[id].tReceive > msgTimeout) or
        (states[id].role >=ROLE_RESERVE_PLAYER) or 
        (states[id].role ==0) then
        eta[id] = math.huge;
        ddefend[id] = math.huge;
      end

    end
  end


  --For defender behavior testing
  force_defender = Config.team.force_defender or 0;
  if force_defender == 1 then
    gcm.set_team_role(ROLE_DEFENDER);
  elseif force_defender ==2 then
    gcm.set_team_role(ROLE_DEFENDER2);
  end

  if role ~= gcm.get_team_role() then
    set_role(gcm.get_team_role());
  end

  --Only switch role during gamePlaying state
  --If role is forced for testing, don't change roles


--[[
    print('---------------');
    for id=1,5 do
      print(id,roles[id],eta[id],ddefend[id])
    end
    print('---------------');
--]]



  if gcm.get_game_state()==3 and force_defender ==0 then
    -- goalie and reserve player never changes role
    if role~=ROLE_GOALIE and role<ROLE_RESERVE_PLAYER then 
      minETA, minEtaID = util.min(eta);
      if minEtaID == playerID then --Lowest ETA : attacker
        set_role(ROLE_ATTACKER);
      else
        -- furthest player back is defender
        maxDDefID = 0;
        maxDDef = 0;

        minDDefID = 0;
        minDDef = math.huge;

        --Find the player most away from the defending goal
        --TODO: 2nd defender 
        for id = 1,5 do
          --goalie, current attacker and and reserve don't count
          if id ~= minEtaID and 	  
            roles[id]~=ROLE_ATTACKER and 
            roles[id]<ROLE_RESERVE_PLAYER then 
	    --Dead players have infinite ddefend
            if ddefend[id] > maxDDef and ddefend[id]<20.0 then
              maxDDefID = id;
              maxDDef = ddefend[id];
            end
            if ddefend[id] < minDDef then
              minDDefID = id;
              minDDef = ddefend[id];
            end
          end
        end
--	print("min max",minDDefID, maxDDefID)
        if maxDDefID == minDDefID then --only one player, go defend
          set_role(ROLE_DEFENDER)
        elseif maxDDefID == playerID then --the player most away from our goal
          set_role(ROLE_SUPPORTER);    -- support
        else --other players go defend
          set_role(ROLE_DEFENDER);    -- defense 
	  --TODO: WHICH defender?

        end
      end
    end
    --We assign role based on player ID during initial and ready state
  elseif gcm.get_game_state()<2 then 
    if role==ROLE_ATTACKER then
      --Check whether there are any other attacker with smaller playerID
      role_switch = false;
      for id=1,5 do
        if roles[id]==ROLE_ATTACKER and id<playerID then
          role_switch = true;
        end
      end
      if role_switch then set_role(ROLE_DEFENDER);end --Switch to defender
    end
    if role==ROLE_DEFENDER then
      --Check whether there are any other depender with smaller playerID
      role_switch = false;
      for id=1,5 do
        if roles[id]==ROLE_DEFENDER and id<playerID then
          role_switch = true;
        end
      end
      if role_switch then set_role(ROLE_SUPPORTER);end --Switch to supporter
    end
  end
  update_shm() 
  update_teamdata();
  update_obstacle();
  check_flip();
end

function update_teamdata()
  attacker_eta = math.huge;
  defender_eta = math.huge;
  defender2_eta = math.huge;
  supporter_eta = math.huge;
  goalie_alive = 0; 

  attacker_pose = {0,0,0};
  defender_pose = {0,0,0};
  defender2_pose = {0,0,0};
  supporter_pose = {0,0,0};
  goalie_pose = {0,0,0};

  best_scoreBall = 0;
  best_ball = {0,0,0};
  for id = 1,5 do
    --Update teammates pose information
    if states[id] and states[id].tReceive and
      (t - states[id].tReceive < msgTimeout) then

      --Team ball calculation here
      --Score everyone's ball position info and pick the best one
      if id~=playerID and states[id].role<4 then
        rBall = math.sqrt(states[id].ball.x^2 + states[id].ball.y^2);
        tBall = states[id].time - states[id].ball.t;
        pBall = states[id].ball.p;
        scoreBall = pBall * 
        math.exp(-rBall^2 / 12.0)*
        math.max(0,1.0-tBall);
        --print(string.format("r%.1f t%.1f p%.1f s%.1f",rBall,tBall,pBall,scoreBall))
        if scoreBall > best_scoreBall then
          best_scoreBall = scoreBall;
          posexya=vector.new( 
            {states[id].pose.x, states[id].pose.y, states[id].pose.a} );
          best_ball=util.pose_global(
            {states[id].ball.x,states[id].ball.y,0}, posexya);
        end
      end

      if states[id].role==ROLE_GOALIE then
        goalie_alive =1;
        goalie_pose = {
          states[id].pose.x,states[id].pose.y,states[id].pose.a};

        goalie_ball = util.pose_global(
          {states[id].ball.x,states[id].ball.y,0},
	  goalie_pose);
        goalie_ball[3] = states[id].ball.t;	

      elseif states[id].role==ROLE_ATTACKER then
          attacker_pose = {states[id].pose.x,states[id].pose.y,states[id].pose.a};
          attacker_eta = eta[id];
      elseif states[id].role==ROLE_DEFENDER then
          defender_pose = {states[id].pose.x,states[id].pose.y,states[id].pose.a};
          defender_eta = eta[id];
      elseif states[id].role==ROLE_SUPPORTER then
          supporter_eta = eta[id];
          supporter_pose = {states[id].pose.x,states[id].pose.y,states[id].pose.a};
      end
    end
  end

  wcm.set_robot_team_ball(best_ball);
  wcm.set_robot_team_ball_score(best_scoreBall);

  wcm.set_team_attacker_eta(attacker_eta);
  wcm.set_team_defender_eta(defender_eta);
  wcm.set_team_supporter_eta(supporter_eta);
  wcm.set_team_defender2_eta(defender2_eta);
  wcm.set_team_goalie_alive(goalie_alive);

  wcm.set_team_attacker_pose(attacker_pose);
  wcm.set_team_defender_pose(defender_pose);
  wcm.set_team_goalie_pose(goalie_pose);
  wcm.set_team_supporter_pose(supporter_pose);
  wcm.set_team_defender2_pose(defender2_pose);

end

function exit() end
function get_role()   return role; end
function get_player_id()    return playerID; end
function update_shm() gcm.set_team_role(role);end

function set_role(r)
  if role ~= r then 
    role = r;
    Body.set_indicator_role(role);
    if role == ROLE_ATTACKER then  Speak.talk('Attack');
    elseif role == ROLE_DEFENDER then  Speak.talk('Defend');
    elseif role == ROLE_SUPPORTER then Speak.talk('Support');
    elseif role == ROLE_GOALIE then Speak.talk('Goalie');
    elseif role == ROLE_DEFENDER2 then Speak.talk('Defender Two')
    elseif role == ROLE_RESERVE_PLAYER then Speak.talk('Player waiting')
    elseif role == ROLE_RESERVE_GOALIE then Speak.talk('Goalie waiting')
    else Speak.talk('ERROR: Unknown Role');
    end
  end
  update_shm();
end

function pack_labelB()
  labelB = vcm.get_image_labelB();
  width = vcm.get_image_width()/8; 
  height = vcm.get_image_height()/8;
  count = vcm.get_image_count();
  array = serialization.serialize_label_rle(
    labelB, width, height, 'uint8', 'labelB',count);
  state.labelB = array;
end

function pack_vision_info()
  --Added Vision Info 
  state.goal=0;
  state.goalv1={0,0};
  state.goalv2={0,0};
  if vcm.get_goal_detect()>0 then
    state.goal = 1 + vcm.get_goal_type();
    local v1=vcm.get_goal_v1();
    local v2=vcm.get_goal_v2();
    state.goalv1[1],state.goalv1[2]=v1[1],v1[2];
    state.goalv2[1],state.goalv2[2]=0,0;
    centroid1 = vcm.get_goal_postCentroid1();
    orientation1 = vcm.get_goal_postOrientation1();
    axis1 = vcm.get_goal_postAxis1();
    state.goalB1 = {centroid1[1],centroid1[2],
    orientation1,axis1[1],axis1[2]};
    if vcm.get_goal_type()==3 then --two goalposts 
      state.goalv2[1],state.goalv2[2]=v2[1],v2[2];
      centroid2 = vcm.get_goal_postCentroid2();
      orientation2 = vcm.get_goal_postOrientation2();
      axis2 = vcm.get_goal_postAxis2();
      state.goalB2 = {centroid2[1],centroid2[2],
      orientation2,axis2[1],axis2[2]};
    end
  end
  state.landmark=0;
  state.landmarkv={0,0};
  state.corner=0;
  state.cornerv={0,0};
  if vcm.get_corner_detect()>0 then
    state.corner = vcm.get_corner_type();
    local v = vcm.get_corner_v();
    state.cornerv[1],state.cornerv[2]=v[1],v[2];
  end
end

function check_flip()
  if flip_correction ==0 then return; end
  if role==0 then return; end

  --print("Goalie ball");
  --util.ptable(goalie_ball);
  --print("Player ball");
  --util.ptable(state_ball);
  --d = math.sqrt((-state_ball[1] - goalie_ball[1])^2 + (-state_ball[2] $

  pose = wcm.get_pose();
  ball = wcm.get_ball();


  ball_global = util.pose_global({ball.x,ball.y,0},{pose.x,pose.y,pose.a});

  ball_flip_dist_threshold = 2.0;
  ball_flip_x_threshold = 1.0;
  ball_flip_y_threshold = 0.6;
  ball_flip_t_threshold = 0.5; --Both robot should be looking at the ball
  local dist_balls = math.abs(ball_global[1]-goalie_ball[1]);

--[[
  print(string.format("Goalie ball: %.2f %.2f Ball: %.2f %.2f",
	goalie_ball[1],goalie_ball[2],
	ball_global[1],ball_global[2]

  ));
--]]
  --util.ptable(goalie_ball);
  --print("Player ball");


  if math.abs(ball.x) > ball_flip_x_threshold and
     ball_global[1]*goalie_ball[1] < 0 and
     ball.t < ball_flip_t_threshold and
     goalie_ball[3] < ball_flip_t_threshold and
     goalie_alive >0 and
     dist_balls > ball_flip_dist_threshold then

    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    print("FLIP DETECTED, CORRECTING");
    wcm.set_robot_flipped(1);
  end
end



--NSL role can be set arbitarily, so use config value
set_role(Config.game.role or 1);
