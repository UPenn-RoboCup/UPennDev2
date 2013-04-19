module(..., package.seeall);

local Body = require('Body')
local fsm = require('fsm')
local gcm = require('gcm')

local bodyIdle = require('bodyIdle')
local bodyStart = require('bodyStart')
local bodyStop = require('bodyStop')
local bodyReady = require('bodyReady')
local bodySearch = require('bodySearch')
local bodyApproach = require('bodyApproach')
local bodyKick = require('bodyKick')
local bodyPosition = require('bodyPosition')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);
sm:add_state(bodySearch);
sm:add_state(bodyApproach);
sm:add_state(bodyKick);
sm:add_state(bodyPosition);

sm:set_transition(bodyStart, 'done', bodyPosition);

sm:set_transition(bodyPosition, 'timeout', bodyPosition);
sm:set_transition(bodyPosition, 'ballLost', bodySearch);
sm:set_transition(bodyPosition, 'ballClose', bodyApproach);

sm:set_transition(bodySearch, 'ball', bodyPosition);
sm:set_transition(bodySearch, 'timeout', bodyPosition);

sm:set_transition(bodyApproach, 'ballFar', bodyPosition);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'timeout', bodyPosition);
sm:set_transition(bodyApproach, 'kick', bodyKick);

sm:set_transition(bodyKick, 'done', bodyPosition);

sm:set_transition(bodyPosition, 'fall', bodyPosition);
sm:set_transition(bodyApproach, 'fall', bodyPosition);
sm:set_transition(bodyKick, 'fall', bodyPosition);

-- set state debug handle to shared memory settor
sm:set_state_debug_handle(gcm.set_fsm_body_state);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
