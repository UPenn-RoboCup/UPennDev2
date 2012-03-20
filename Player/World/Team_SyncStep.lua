module(..., package.seeall);
--by SJ, 0215

require('Body');
require('World');
require('Comm');
require('vector');
require('Config')
require('serialization')
require('vcm')
require 'walk'

playerID = Config.game.playerID;
role = Config.game.role;
task = 0; -- Circling to start

count = 0;
state = {};
state.id = playerID;
state.time = Body.get_time();
state.role = role;
state.task = task;
state.step = walk.supportLeg;
--state.phase = walk.getPhase( Body.get_time() );
state.phase = 0;

state.tStep = walk.tStep
state.stretcher = 1;

states = {};
myvel = {0,0,0};
myPhase = 0;

updateInterval = 2;
team_sm = 0; -- for use by test_vision

-- Out of Step Phase [0,tStep]
--mytStep = .3
omega = 0.05;
--K_change = .2;
K_change = .2;

function processMessage( t )

  if( t.id == 1) then
    print("controller command!")
    if(t.cmd==3) then
      pickup.throw=0;
      Motion.event("pickup");   
    elseif(t.cmd==2) then -- move
      print("Target Vel: ", unpack(t.targetvel) )
      myvel = t.targetvel;
      if(playerID==2) then 
        myvel[1] = -1*myvel[1];
        myvel[2] = -1*myvel[2];
        myvel[3] = -1*myvel[3];
      end
    elseif(t.cmd == 1) then -- stand in place
      walk.stopAlign();
      Motion.event("standup");
    elseif(t.cmd==4) then --sit down
      Motion.event("sit");
    elseif(t.cmd==5) then
      Motion.event("walk");
      walk.start();
    elseif(t.cmd==6) then
      -- Start the state machine and stop it
      team_sm = 1 - team_sm;
    end
    -- Done processing the controller
    return;
  end

  -- If I am master, then return
  if( playerID==2 ) then
    return;
  end
  
  -- Act on the step changes now...
  tDiff = 0;

  myStep = walk.supportLeg;

  -- Add the continuous phase lag/leading
--[[
  myPhase = myPhase + omega;
  if( myPhase > walk.tStep ) then
    --print("Omega: "..omega);
    --print("Before: "..myPhase..", "..mywalk.." walk.tStep: "..walk.tStep);
    myStep = 1-myStep;
    myPhase = myPhase - walk.tStep;
    --print("After: "..myPhase..", "..mywalk.." walk.tStep: "..walk.tStep);
  elseif( myPhase<0 ) then
    myStep = 1-myStep;
    myPhase = myPhase + walk.tStep;
  end
--]]
--[[
  t.phase = t.phase + omega;
  if( t.phase > t.tStep ) then
    t.step = 1-t.step;
    myPhase = myPhase - t.tStep;
  elseif( myPhase<0 ) then  
    t.step = 1-t.step;
    t.phase = t.phase + t.tStep;
  end
--]]
  if( t.step ~= myStep ) then --Same step (==) / Opposite step (~=)
    tDiff = myPhase - t.phase;
    --print("Same step, difference: "..tDiff)
  else -- Is it better to slow down or speed up?
    lead = myPhase + (t.tStep-t.phase);
    lag  = t.phase + (walk.tStep-myPhase);
    --print("Out of step: Lead ("..lead..") Lag ("..lag..")");
    if( lead<lag ) then
      --print("Lead! "..lead);
      if( walk.tStep > t.tStep ) then -- then I am ok
      else
        tDiff = lead;
      end
    else
      --print("Lag! "..lag);
      if( walk.tStep < t.tStep ) then -- then I am ok
      else
        tDiff = -1*lag;  -- Lag is always negative
      end
    end
  end

  tChange = K_change * tDiff;
  --print( "tDiff: ", tDiff );
  
  walk.tStep = t.tStep + tChange; -- Should be plus! increase tStep decrease step time
  --print("tChange: "..tChange);
  if( walk.tStep<0.2 ) then
    walk.tStep = 0.2;
  elseif( walk.tStep>.75 ) then
    walk.tStep = .75;
  end

end

function entry()
end

function update()
        walk.set_velocity(unpack(myvel));
--  print("SyncStep technology");
  count = count + 1;
--  myPhase = walk.getPhase( Body.get_time() );
myPhase = 0;

  -- Update my state
  state.time = Body.get_time();

  -- Dealing with Communications
  if (math.mod(count, updateInterval) == 0) then
    while Comm.size()>0 do
      rcv=Comm.receive();
      if string.byte(rcv)==123 then --"{"
        local t = serialization.deserialize(rcv);
        if (t and (t.id) and (t.id ~= playerID)) then
          states[t.id] = t;
          processMessage( t );
        end
      end
    end -- while

    state.task = task;
    state.role = role;
    state.step = walk.supportLeg;
    state.phase = myPhase + 0.002 + omega; -- Add 2ms transport time.  Sampl rate of 100Hz (2x this)
    state.tStep = walk.tStep

    Comm.send(serialization.serialize(state));
    --Copy of message sent out to other players
    states[playerID] = state;
  end


  for id=2,3 do
    if (id~=playerID and not states[id]) then
      return;
    end
  end


  -- Look at the other person only
  for id=2,3 do
    if ( id ~= playerID ) then
      -- If the other bot is waiting and so are we, then pick up!
      -- If they are picking up, then we pick up, too (prevents a missed opportunity)
      if( (states[id].task==1 or states[id].task==2) and state.task==1 ) then
        Speak2.talk('Pick up!');
        task = 2;
      -- Same thing, but for walking
      elseif( (states[id].task==3 or states[id].task==4) and state.task==3 ) then
        Speak2.talk('Now walk it out!');
        task = 4;
      end
    end
  end

end

function exit()
end

function get_role()
  return role;
end

function get_task()
  return task;
end

function setTask( k )
  task = k;
end

