module(..., package.seeall);

local Body = require('Body')
local fsm = require('fsm')
local gcm = require('gcm')
local Config = require('Config')

local bodyIdle = require('bodyIdle')
local bodyStart = require('bodyStart')
local bodyStop = require('bodyStop')
local bodyReady = require('bodyReady')

local bodySearch = require('bodySearch')
local bodyApproach = require('bodyApproach')
local bodyDribble = require('bodyDribble')
local bodyAlign = require('bodyAlign')
local bodyAlignStep = require('bodyAlignStep')
local bodyKick = require('bodyKick')
local bodyPositionSimple = require('bodyPositionSimple')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

sm:add_state(bodySearch);
sm:add_state(bodyApproach);

sm:add_state(bodyDribble);

sm:add_state(bodyAlign);
sm:add_state(bodyAlignStep);
sm:add_state(bodyKick);
sm:add_state(bodyPositionSimple);


------------------------------------------------------
-- Simpler FSM (bodyChase and bodyorbit)
------------------------------------------------------

sm:set_transition(bodyStart, 'done', bodyPositionSimple);

sm:set_transition(bodyPositionSimple, 'timeout', bodyPositionSimple);
sm:set_transition(bodyPositionSimple, 'ballLost', bodySearch);
sm:set_transition(bodyPositionSimple, 'ballClose', bodyDribble);
sm:set_transition(bodyPositionSimple, 'done', bodyDribble);

sm:set_transition(bodyDribble, 'ballFar', bodyPositionSimple);
sm:set_transition(bodyDribble, 'ballLost', bodySearch);
sm:set_transition(bodyDribble, 'ballClose', bodyAlign);

sm:set_transition(bodySearch, 'ball', bodyPositionSimple);
sm:set_transition(bodySearch, 'timeout', bodySearch);

sm:set_transition(bodyApproach, 'ballFar', bodyPositionSimple);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'timeout', bodyPositionSimple);
--sm:set_transition(bodyApproach, 'kick', bodyKick);
sm:set_transition(bodyApproach, 'kick', bodyAlign);



sm:set_transition(bodyKick, 'timeout', bodyApproach);
sm:set_transition(bodyAlign, 'done', bodyKick);
--sm:set_transition(bodyAlign, 'reposition', bodyApproach);
sm:set_transition(bodyAlign, 'reposition', bodyAlignStep);

sm:set_transition(bodyAlignStep, 'done', bodyAlign);

sm:set_transition(bodyKick, 'timeout', bodyPositionSimple);
sm:set_transition(bodyKick, 'done', bodyPositionSimple);


sm:set_transition(bodyAlign, 'fall', bodyPositionSimple);
sm:set_transition(bodyPositionSimple, 'fall', bodyPositionSimple);
sm:set_transition(bodyApproach, 'fall', bodyPositionSimple);
sm:set_transition(bodyKick, 'fall', bodyPositionSimple);

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
