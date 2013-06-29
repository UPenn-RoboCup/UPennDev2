module(..., package.seeall);

require('Body')
require('walk')
require('BodyFSM')
require('HeadFSM')
require('Speak')
require('vector')
require('gcm')
require('BodyFSM')
require('HeadFSM')

t0 = 0;

function entry()
  print(_NAME..' entry');

  t0 = Body.get_time();
  was_penalized = gcm.get_game_was_penalized();
  was_set = gcm.get_game_was_set();

  if was_penalized>0 then
    --Coming back from penalization state
    Speak.talk('Unpenalized');
    BodyFSM.sm:set_state('bodyUnpenalized');
    HeadFSM.sm:set_state('headStart');
  else
    if was_set>0 then
      --This robot was correctly initialized through gameSet state
      --Need to do 10-second wait and everything
      Speak.talk('Playing');
      BodyFSM.sm:set_state('bodyStart');
      HeadFSM.sm:set_state('headStart');
    else
      --This robot is just turned on 
      --initialize particles to correct position
      BodyFSM.sm:set_state('bodyUnpenalized');
      HeadFSM.sm:set_state('headStart');
    end
  end
  gcm.set_game_was_penalized(0);
  gcm.set_game_was_set(0);
  -- set indicator
  Body.set_indicator_state({0,1,0});
end

function update()
  local state = gcm.get_game_state();

  if (state == 0) then
    return 'initial';
  elseif (state == 1) then
    return 'ready';
  elseif (state == 2) then
    return 'set';
  elseif (state == 4) then
    return 'finished';
  end

  -- check for penalty 
  if gcm.in_penalty() then
    return 'penalized';
  end
end

function exit()
end
