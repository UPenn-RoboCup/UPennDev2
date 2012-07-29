module(..., package.seeall);

require('Body')
require('Motion')
require 'walk'
require 'Speak'
require 'gcm'

t0 = 0;

playerID = Config.game.playerID;

function entry()
  buttonPressed = 0;
  print(_NAME..' entry');
  t0 = Body.get_time();

  Motion.event("sit");
  --  Motion.sm:set_state('resit');
  --  Motion.sm:set_state('standstill');
end

function update()
  Motion.event("sit");
  t = Body.get_time();

  -- Toggle stabilization when idle and penalized
  if( gcm.in_penalty() ) then
    if( Body.get_change_role()==0 ) then
      -- toggle when button is released
      if buttonPressed == 1 then
        if(walk.no_stabilize) then
          Speak.talk('Enabling stabilization!')
        else
          Speak.talk('Disabling stabilization!')
        end
        walk.no_stabilize = not walk.no_stabilize;
        buttonPressed = 0;
      end
    else
      buttonPressed = 1;
    end
  end
end

function exit()
  Motion.sm:set_state('stance');
end
