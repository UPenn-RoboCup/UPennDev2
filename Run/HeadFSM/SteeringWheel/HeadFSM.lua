module(..., package.seeall);

require('fsm')
require('scm')

require('headFixed')
require('headTeleop')
require('headScan')
require('headSlowScan')


sm = fsm.new(headTeleop);
sm:add_state(headFixed);
sm:add_state(headScan);
sm:add_state(headSlowScan);

sm:set_transition(headFixed, 'teleop', headTeleop);
sm:set_transition(headFixed, 'scan', headScan);
sm:set_transition(headFixed, 'slowscan', headSlowScan);

sm:set_transition(headScan, 'done', headTeleop);
sm:set_transition(headSlowScan, 'done', headTeleop);

sm:set_transition(headTeleop, 'fixed', headFixed);
sm:set_transition(headTeleop, 'scan', headScan);
sm:set_transition(headTeleop, 'slowscan', headSlowScan);




function entry()
  sm:entry();
end

function update()
  sm:update();
end

function exit()
  sm:exit()
end
