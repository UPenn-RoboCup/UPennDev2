module(..., package.seeall);

local fsm = require('fsm')
local bodyIdle = require('bodyIdle')
local bodyChase = require('bodyChase')
local bodySearch = require('bodySearch')
local bodyApproach = require('bodyApproach')
local bodyPickup = require('bodyPickup')
--local bodyFaceOff = require('bodyFaceOff');

sm = fsm.new(bodyIdle);
sm:add_state(bodyChase);
sm:add_state(bodySearch);
sm:add_state(bodyApproach);
sm:add_state(bodyPickup);
--sm:add_state(bodyFaceOff);

-- Search for the stretcher
sm:set_transition(bodySearch, 'stretcher', bodyChase);
sm:set_transition(bodySearch, 'timeout', bodySearch);

-- Chase after the stretcher
sm:set_transition(bodyChase, 'stretcherLost', bodySearch);
sm:set_transition(bodyChase, 'stretcherClose', bodyApproach);
sm:set_transition(bodyChase, 'timeout', bodyChase);

-- Approach the stretcher (et into position)
sm:set_transition(bodyApproach, 'stretcherFar', bodyChase);
sm:set_transition(bodyApproach, 'stretcherLost', bodySearch);
sm:set_transition(bodyApproach, 'pickup', bodyPickup);
sm:set_transition(bodyApproach, 'timeout', bodyChase);

-- Pickup the stretcher
--sm:set_transition(bodyPickup, 'done', bodyFaceOff);

-- Face each other with the stretcher
--sm:set_transition(bodyFaceOff, 'timeout', bodyFaceOff);

-- If you fall, what do you do?
sm:set_transition(bodyChase, 'fall', bodySearch);
sm:set_transition(bodyApproach, 'fall', bodySearch);
sm:set_transition(bodyPickup, 'fall', bodyIdle);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
