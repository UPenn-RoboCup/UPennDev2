module(..., package.seeall);

local HeadFSM = require('HeadFSM')
local BodyFSM = require('BodyFSM')
local Speak = require('Speak')
local vector = require('vector')
local util = require('util')
local gcm = require('gcm')
local BodyFSM = require('BodyFSM')
local HeadFSM = require('HeadFSM')

function entry()
  print(_NAME..' entry');

  HeadFSM.sm:set_state('headIdle');
  BodyFSM.sm:set_state('bodyIdle');

  Speak.talk('Penalized');
	count = 0;
  -- set indicator
  Body.set_indicator_state({1,0,0});
end

function update()
  local change = 0;
  
	if Body.get_change_state()==1 then
		count=count+1;
	else
		count=0;
	end
	
	if count > 80 then
		change = 1;
		count = 0;
	end

  if (change == 1) then
    return 'playing';
  end
end

function exit()
end
