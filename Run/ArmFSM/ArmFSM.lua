module(..., package.seeall);

require('fsm')

require('armIdle')
require('armInit')
require('armReset')

require('armReady')
require('armTeleop')

require('armWheelGrip')
require('armWheelTurn')
require('armWheelRelease')

require('armInitLeft');
require('armReadyLeft');
require('armGrabLeft');
require('armOpenLeft');
require('armResetLeft');




sm = fsm.new(armIdle);
sm:add_state(armInit);
sm:add_state(armReady);
sm:add_state(armReset);

sm:add_state(armWheelGrip);
sm:add_state(armWheelTurn);
sm:add_state(armWheelRelease);

sm:add_state(armInitLeft);
sm:add_state(armReadyLeft);
sm:add_state(armGrabLeft);
sm:add_state(armOpenLeft);
sm:add_state(armResetLeft);

--sm:add_state(armTeleop);

sm:set_transition(armIdle, 'init', armInit);
sm:set_transition(armInit, 'done', armReady);
sm:set_transition(armReady, 'wheelgrab', armWheelGrip);


sm:set_transition(armWheelGrip, 'reset', armWheelRelease);
sm:set_transition(armWheelGrip, 'done', armWheelTurn);
sm:set_transition(armWheelGrip, 'stop', armReady);
sm:set_transition(armWheelGrip, 'reset', armWheelRelease);

sm:set_transition(armWheelTurn, 'reset', armWheelRelease);
sm:set_transition(armWheelTurn, 'stop', armWheelRelease);

sm:set_transition(armWheelRelease, 'done', armReady);
sm:set_transition(armReady, 'reset', armReset);

sm:set_transition(armReset, 'done', armIdle);



function entry()
  sm:entry();
end

function update()
  sm:update();
end

function exit()
  sm:exit()
end
