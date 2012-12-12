cwd = os.getenv('PWD')
require('init')

require('unix')
require('Config')
require('shm')
require('vector')
require('mcm')
require('Speak')
require('getch')
require('Body')

require 'Comm'
wired = true
-- Initialization
if( wired ) then
  print("My address:",Config.dev.ip_wired)
  Comm.init(Config.dev.ip_wired,Config.dev.ip_wired_port)
else
  print("My address:",Config.dev.ip_wireless)
  Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);
end

darwin = false;
webots = false;


initToggle = true;
targetvel=vector.zeros(3);
button_pressed = {0,0};


function process_keyinput()
  local str=getch.get();
  if #str>0 then
    local byte=string.byte(str,1);
    -- Walk velocity setting
    if byte==string.byte("i") then	targetvel[1]=targetvel[1]+0.02;
    elseif byte==string.byte("j") then	targetvel[3]=targetvel[3]+0.1;
    elseif byte==string.byte("k") then	targetvel[1],targetvel[2],targetvel[3]=0,0,0;
    elseif byte==string.byte("l") then	targetvel[3]=targetvel[3]-0.1;
    elseif byte==string.byte(",") then	targetvel[1]=targetvel[1]-0.02;
    elseif byte==string.byte("h") then	targetvel[2]=targetvel[2]+0.02;
    elseif byte==string.byte(";") then	targetvel[2]=targetvel[2]-0.02;

    elseif byte==string.byte("1") then	
      kick.set_kick("kickForwardLeft");
      Motion.event("kick");
    elseif byte==string.byte("2") then	
      kick.set_kick("kickForwardRight");
      Motion.event("kick");
    elseif byte==string.byte("3") then	
      kick.set_kick("kickSideLeft");
      Motion.event("kick");
    elseif byte==string.byte("4") then	
      kick.set_kick("kickSideRight");
      Motion.event("kick");
    elseif byte==string.byte("5") then
      walk.doWalkKickLeft();
    elseif byte==string.byte("6") then
      walk.doWalkKickRight();
    elseif byte==string.byte("t") then
      walk.doSideKickLeft();
    elseif byte==string.byte("y") then
      walk.doSideKickRight();


    elseif byte==string.byte("w") then
      Motion.event("diveready");
    elseif byte==string.byte("a") then
      dive.set_dive("diveLeft");
      Motion.event("dive");
    elseif byte==string.byte("s") then
      dive.set_dive("diveCenter");
      Motion.event("dive");
    elseif byte==string.byte("d") then
      dive.set_dive("diveRight");
      Motion.event("dive");

--[[
	elseif byte==string.byte("z") then
		grip.throw=0;
		Motion.event("pickup");
	elseif byte==string.byte("x") then
		grip.throw=1;
		Motion.event("throw");
--]]

	elseif byte==string.byte("z") then
	    walk.startMotion("hurray1");

	elseif byte==string.byte("x") then
	    walk.startMotion("hurray2");

	elseif byte==string.byte("c") then
	    walk.startMotion("swing");

	elseif byte==string.byte("v") then
	    walk.startMotion("2punch");

--	elseif byte==string.byte("b") then
--	    walk.startMotion("point");



	elseif byte==string.byte("b") then
	    grip.throw=0;
	    Motion.event("pickup");
	elseif byte==string.byte("n") then
	    grip.throw=1;
	    Motion.event("pickup");

    elseif byte==string.byte("7") then	
      Motion.event("sit");
    elseif byte==string.byte("8") then	
      if walk.active then walk.stop();end
      Motion.event("standup");
    elseif byte==string.byte("9") then	
      Motion.event("walk");
      walk.start();
    end
    walk.set_velocity(unpack(targetvel));
    print("Command velocity:",unpack(walk.velCommand))
  end
end

-- main loop
count = 0;
lcount = 0;
tUpdate = unix.time();

function update()
Body.set_larm_hardness(vector.new({.5,.5,.5,.5}))
--Body.set_larm_hardness(vector.new({0,0,0,0}))
  count = count + 1;
  -- update state machines 
  process_keyinput();

-- Receive over the Comm
  while (Comm.size() > 0) do
    msg = Comm.receive()
--[[
    if compressPick then
      msg = Z.uncompress(msg, #msg)
    end
--]]
    t = serialization.deserialize(msg);
    if t  then
--print('Update!')
      t.tReceive = unix.time();
      state = t;
Body.set_larm_command(vector.new(t.qL))
    end
  end
--local jangles = {0,0,0,0}
--Body.set_larm_command(math.pi/180*vector.new(jangles))
  
  local dcount = 50;
  if (count % 50 == 0) then
    print('fps: '..(dcount / (unix.time() - tUpdate)));
    tUpdate = unix.time();
    -- update battery indicator
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end

  -- check if the last update completed without errors
  lcount = lcount + 1;
  if (count ~= lcount) then
    print('count: '..count)
    print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end

end

  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  while 1 do
    update();
    unix.usleep(tDelay);
  end
