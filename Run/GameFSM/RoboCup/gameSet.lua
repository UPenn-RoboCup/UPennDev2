module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local Speak = require('Speak')
local vector = require('vector')
local gcm = require('gcm')
local wcm = require('wcm')
local BodyFSM = require('BodyFSM')
local HeadFSM = require('HeadFSM')

t0 = 0;
timeout = 5.0;

function entry()
  print(_NAME..' entry');

  mcm.set_motion_fall_check(0) --disable fall
  t0 = Body.get_time();

  -- stop walking and wait for game to start
  BodyFSM.sm:set_state('bodyStop');
  HeadFSM.sm:set_state('headTrack');

  Speak.talk('Set');

  -- set indicator
  Body.set_indicator_state({1,1,0});
end

function update()

  t = Body.get_time();
  --Update kickoff timer
  if gcm.get_game_kickoff()>0 then
    wcm.set_kick_tKickOff(t);
    wcm.set_kick_kickOff(1);
  end


  local state = gcm.get_game_state();

  -- stop walk (in case getup routine is invoked)
  walk.stop();

  if (state == 0) then
    return 'initial';
  elseif (state == 1) then
    return 'ready';
  elseif (state == 3) then
    return 'playing';
  elseif (state == 4) then
    return 'finished';
  end
  
  -- check for penalty
  if gcm.in_penalty() then
    return 'penalized';
  end
end

function exit()
  mcm.set_motion_fall_check(1) --re-enable fall
end
