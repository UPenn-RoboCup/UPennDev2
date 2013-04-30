module(..., package.seeall);

local fsm = require('fsm')

local bodyIdle = require('bodyIdle')
local bodyLearnLUT = require('bodyLearnLUT')
local bodyPosition = require('bodyPosition')
local bodyWait = require('bodyWait')
local bodyStop = require('bodyStop')
local bodyApproachGoal = require('bodyApproachGoal')
local bodyAvoid = require('bodyAvoid')

sm = fsm.new(bodyIdle);
sm:add_state(bodyPosition);
sm:add_state(bodyWait);
sm:add_state(bodyStop);
sm:add_state(bodyAvoid);
sm:add_state(bodyApproachGoal);


-- Obstacle Challenge FSM
sm:set_transition(bodyWait, 'done', bodyApproachGoal);
sm:set_transition(bodyWait, 'timeout', bodyWait);

sm:set_transition(bodyApproachGoal, 'timeout', bodyApproachGoal)
sm:set_transition(bodyApproachGoal, 'obstacle', bodyAvoid)
sm:set_transition(bodyApproachGoal, 'done', bodyStop)

sm:set_transition(bodyAvoid, 'done', bodyApproachGoal)
sm:set_transition(bodyAvoid, 'timeout', bodyApproachGoal)
sm:set_transition(bodyAvoid, 'obstacle', bodyAvoid)

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
