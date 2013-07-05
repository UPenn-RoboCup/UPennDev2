module(..., package.seeall);

require('fsm')

require('bodyTeleop')
require('bodyIdle')
require('bodyNavigate')


sm = fsm.new(bodyTeleop);
sm:add_state(bodyIdle);
sm:add_state(bodyNavigate);

sm:set_transition(bodyTeleop, 'stop', bodyIdle);
sm:set_transition(bodyTeleop, 'navigate', bodyNavigate);

sm:set_transition(bodyNavigate, 'done', bodyTeleop);
sm:set_transition(bodyNavigate, 'teleop', bodyTeleop);
sm:set_transition(bodyNavigate, 'stop', bodyIdle);

sm:set_transition(bodyIdle, 'teleop', bodyTeleop);
sm:set_transition(bodyIdle, 'navigate', bodyNavigate);




function entry()
  sm:entry();
end

function update()
  sm:update();
end

function exit()
  sm:exit()
end
