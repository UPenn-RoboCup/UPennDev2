module(..., package.seeall);

local Body = require('Body')
local fsm = require('fsm')
local gcm = require('gcm')
local Config = require('Config')


local bodyIdle = require('bodyIdle')
local bodyStart = require('bodyStart')
local bodyStop = require('bodyStop')
local bodyReady = require('bodyReady')

local bodyStepOut = require('bodyStepOut')
local bodyPositionGoalie = require('bodyPositionGoalie')
local bodyAnticipate = require('bodyAnticipate')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

sm:add_state(bodyStepOut);
sm:add_state(bodyPositionGoalie);
sm:add_state(bodyAnticipate);

sm:set_transition(bodyStart, 'done', bodyStepOut);

sm:set_transition(bodyStepOut, 'done', bodyAnticipate);
sm:set_transition(bodyAnticipate,'done',bodyPositionGoalie);
sm:set_transition(bodyPositionGoalie, 'fall', bodyPositionGoalie);
sm:set_transition(bodyPositionGoalie, 'done', bodyAnticipate);

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
