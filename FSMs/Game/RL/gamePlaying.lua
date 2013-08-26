module(..., package.seeall);

local Body = require('Body')
local BodyFSM = require('BodyFSM')
local HeadFSM = require('HeadFSM')
local Speak = require('Speak')

t0 = 0;

function entry()
  print(_NAME..' entry');

  t0 = Body.get_time();

  BodyFSM.sm:set_state('bodySearch');
  HeadFSM.sm:set_state('headScan');

  Speak.talk('Playing');
	count = 0;
  -- set indicator
  Body.set_indicator_state({0,1,0});

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
    return 'penalized';
  end
end

function exit()
end
