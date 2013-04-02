cwd = os.getenv('PWD')
require('init')

require('Config')
require('shm')
require('vector')
require('Motion')
require('walk')
require('Body')
require("getch")
require('kick')
require('Speak')
require('Comm')

webots = false;
darwin = false;


-- initialize state machines
Motion.entry();

walk.stop();
targetvel=vector.new({0,0,0});
headangle=vector.new({0,0});

walkKick=true;

--Adding head movement && vision...--
Body.set_head_hardness({0.4,0.4});

-- main loop
local tUpdate = unix.time();
local count=0;
local countInterval=1000;
count_dcm=0;
t0=unix.time();
init = false;
calibrating = false;
ready = false;
calibrated=false;

function update()

  count = count + 1;
  
	if (not init)  then
    if (calibrating) then
      if (Body.calibrate(count)) then
        Speak.talk('Calibration done');
        calibrating = false;
        ready = true;
        Comm.init("192.168.1.255", 54321);
      end
      
    elseif (ready) then
      init = true;
    else
      if (count % 20 == 0) then
        if calibrated==false then
          Speak.talk('Calibrating');
          calibrating = true;
          calibrated=true;
        elseif (Body.get_change_role() == 1) then
          smindex = (smindex + 1) % #Config.fsm.body;
        end
      end

      -- toggle state indicator
      if (count % 100 == 0) then
        initToggle = not initToggle;
        if (initToggle) then
          Body.set_indicator_state({1,1,1}); 
        else
          Body.set_indicator_state({0,0,0});
        end
      end
    end
  end

  t=unix.time();
  Motion.update();
  Body.set_head_hardness(0.2);
end
