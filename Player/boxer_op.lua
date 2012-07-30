cwd = os.getenv('PWD')
require('init')

require('unix')
require('Config')
require('shm')
require('vector')
require('vcm')
require('gcm')
require('wcm')
require('mcm')
require('Speak')
require('getch')
require('Body')
require('Motion')

gcm.say_id();

Motion.entry();

darwin = false;
webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
  --SJ: OP specific initialization posing (to prevent twisting)
  Body.set_body_hardness(0.3);
  Body.set_actuator_command(Config.stance.initangle)
  unix.usleep(1E6*0.5);
  Body.set_body_hardness(0);
  Body.set_lleg_hardness({0.6,0.6,0.6,0,0,0});
  Body.set_rleg_hardness({0.6,0.6,0.6,0,0,0});
end 


-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  webots = true;
end

init = false;
calibrating = false;
ready = false;
if( webots or darwin) then
  ready = true;
end

smindex = 0;
initToggle = true;

--SJ: Now we use a SINGLE state machine for goalie and attacker
package.path = cwd..'/BodyFSM/'..Config.fsm.body[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/HeadFSM/'..Config.fsm.head[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/GameFSM/'..Config.fsm.game..'/?.lua;'..package.path;
require('BodyFSM')
require('HeadFSM')
require('GameFSM')

BodyFSM.entry();
HeadFSM.entry();
GameFSM.entry();

-- main loop
count = 0;
lcount = 0;
tUpdate = unix.time();

--Start with PAUSED state
gcm.set_team_forced_role(0); --Don't force role
gcm.set_game_paused(0);
waiting = 0;
if Config.game.role==1 then
  cur_role = 1; --Attacker
else
  cur_role = 0; --Default goalie
end



button_role,button_state = 0,0;
tButtonRole = 0;

function update()
  count = count + 1;
  t = Body.get_time();
  --Update battery info
  wcm.set_robot_battery_level(Body.get_battery_level());
  vcm.set_camera_teambroadcast(1); --Turn on wireless team broadcast


  gcm.set_game_paused(0);
  GameFSM.update();
  BodyFSM.update();
  HeadFSM.update();
  Motion.update();
  Body.update();

  local dcount = 50;
  if (count % 50 == 0) then
    --    print('fps: '..(50 / (unix.time() - tUpdate)));
    tUpdate = unix.time();
    -- update battery indicator
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end

end

-- if using Webots simulator just run update
if (webots) then
  require('cognition');
  cognition.entry();

  -- set game state to Playing
  gcm.set_game_state(3);

  while (true) do
    -- update cognitive process
    cognition.update();
    -- update motion process
    update();

    io.stdout:flush();
  end

end

if( darwin ) then
  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  while 1 do
    update();
    unix.usleep(tDelay);
  end
end
