module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodyStart')
require('bodyBox')
require('bodyReady')
require('bodyStop')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyBox);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

-- Set transitions
sm:set_transition(bodyStart,'done',bodyBox);
sm:set_transition(bodyStart,'timeout',bodyStart);

-- Always box, unless lost data connection / skeleton
sm:set_transition(bodyBox, 'timeout', bodyBox);
sm:set_transition(bodyBox, 'disabled', bodyStart);

-- If you fall, what do you do?
--sm:set_transition(bodyChase, 'fall', bodySearch);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
