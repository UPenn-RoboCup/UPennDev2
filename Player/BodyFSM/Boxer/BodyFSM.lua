module(..., package.seeall);

local fsm = require('fsm')
local bodyIdle = require('bodyIdle')
local bodyStart = require('bodyStart')
local bodyBox = require('bodyBox')
local bodyMimic = require('bodyMimic')
local bodyMimicWalk = require('bodyMimicWalk')
local bodyReady = require('bodyReady')
local bodyStop = require('bodyStop')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyBox);
sm:add_state(bodyMimic);
sm:add_state(bodyMimicWalk);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

-- Set transitions
sm:set_transition(bodyStart,'doneMimic',bodyMimic);
sm:set_transition(bodyStart,'doneMimicWalk',bodyMimicWalk);
sm:set_transition(bodyStart,'doneBox',bodyBox);
sm:set_transition(bodyStart,'timeout',bodyStart);

sm:set_transition(bodyBox, 'disabled', bodyStart);
--sm:set_transition(bodyBox, 'doublepunch', bodyMimic);

sm:set_transition(bodyMimic, 'disabled', bodyStart);
sm:set_transition(bodyMimicWalk, 'disabled', bodyStart);

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
