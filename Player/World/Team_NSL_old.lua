module(..., package.seeall);
--by SJ, 0215

require('Body');
require('Comm');
require('vector');
require('Config')
require('serialization')
require('vcm')


playerID = Config.game.playerID;
role = Config.game.role;

count = 0;
state = {};
state.id = playerID;
state.time = Body.get_time();
state.role = role;
state.pose = {x=0, y=0, a=0};
state.ball = {t=0, x=1, y=0};

states = {};
lastupdate = {0,0,0,0,0};
nwait = 1.5; -- If we don't hear an update for 1.5 seconds, we go to attack mode...

teammate="";
waiting=0;
buddy_id=0;
buddy_pose=vector.new({0,0,0});


function entry()
end

function update()
  count = count + 1;

  -- Fallen or penalized means do not send info over wifi
  if( (vcm.motion.isfalldown[1]==1) or vcm.in_penalty() ) then
    --print("Not sending team since in penalty or fallen down");
    return;
  end

  -- Check the receiving end
  --SJ: I think we should try listening at every frame 

  while Comm.size()>0 do
    rcv=Comm.receive();
    if string.byte(rcv)==123 then --"{"
      t = serialization.deserialize(rcv);
      if (t and (t.id) and (t.id ~= playerID)) then
        --print("Recv mesg from", t.id);
          states[t.id] = t;
          lastupdate[t.id] = Body.get_time();
      end  -- Not our playerID
    end -- Received some string
  end -- while


  -- Check wireless messages every nth count
  -- Updates our knowledge of other robots' states

  local ncount = 10  

  --Goalie and waiting players do not send messages
  if (math.mod(count, ncount) == 0) then
      -- Set our own state
      state.time = Body.get_time();
      state.pose = vcm.get_pose();
      state.ball = vcm.get_ball();
      state.role = role;
      -- Send our state information over wifi

      if role<3 and waiting==0 then 
          Comm.send(serialization.serialize(state));
      end
      -- Keep a copy of message sent out to other players
      states[playerID] = state;
      lastupdate[playerID] = Body.get_time();
  end



  --Clear the recieved state data if they are out of date or penalized

  buddy_id=0;
  teammate="";

  for id=1,5 do
--    print(id,lastupdate[id])
     if lastupdate[id]>0 and lastupdate[id]+nwait < Body.get_time() then
         lastupdate[id]=0;
     end
     if vcm.in_penalty(id) then  
	lastupdate[id]=0;
        teammate=teammate..id..'P ';
     end
     if lastupdate[id]>0 then
	if playerID~=id then 	
	    buddy_id=id;
  	    if states[id].role==1 then
  	       teammate=teammate..id..'A ';
  	    else
	       teammate=teammate..id..'D ';
  	    end
	end
     end     
  end

  --Goalie and waiting players do not switch roles
  if role>2 or waiting==1 then return; end

  --If there are no active robot out there, we are attack by default

  if buddy_id==0 then
      --attack by default
      role = 1;
      return;
  end
  buddy_pose=states[buddy_id].pose;

  --print("Buddy ID",buddy_id)
  --Now we have a active buddy
  --Compare ETA to ball of me and buddy 

  local eta_mine=get_eta(playerID);
  local eta_buddy=get_eta(buddy_id);

--  print("My id "..playerID.."eta"..eta_mine)
--  print("Buddy id "..buddy_id.."eta"..eta_buddy)

  --Role switching
  if eta_buddy<eta_mine and role==1 then
     --Speak2.talk('Defend');
     print("Defend!");
     role = 2;
  elseif eta_buddy>eta_mine and role==2 then
     print("Attack!");
     role = 1;
  end

end

function exit()
end

function get_role()
  return role;
end

function get_eta(id)
    if states[id]==null then
         return 999.0;
    end
    local rBall = math.sqrt(states[id].ball.x^2 + states[id].ball.y^2);
    local tBall = states[id].time - states[id].ball.t;


--print("ID:",id)
--print("rBALL:",rBall);
--states[id].ball.x,states[id].ball.y)


    local eta = rBall/0.10 + 4*math.max(tBall-1.0,0);
    if (states[id].role ~= 1) then
      -- Non attacker penalty:
      eta = eta + 4.0;
    end
    return eta;
end
