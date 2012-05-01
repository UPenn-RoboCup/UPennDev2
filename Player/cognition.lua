module(... or "", package.seeall)

-- Add the required paths
cwd = '.';
computer = os.getenv('COMPUTER') or "";
if (string.find(computer, "Darwin")) then
   -- MacOS X uses .dylib:                                                      
   package.cpath = cwd.."/Lib/?.dylib;"..package.cpath;
else
   package.cpath = cwd.."/Lib/?.so;"..package.cpath;
end
package.path = cwd.."/Util/?.lua;"..package.path;
package.path = cwd.."/Config/?.lua;"..package.path;
package.path = cwd.."/Lib/?.lua;"..package.path;
package.path = cwd.."/Dev/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path; 

require('unix')
require('vcm')
require('gcm')
require('wcm')
require('mcm')
require('Body')

--require('GameControl')

require('Vision')
require('World')
--require('Team')

count = 0;
nProcessedImages = 0;
tUpdate = unix.time();

if (string.find(Config.platform.name,'Webots')) then
  webots = true;
end

function entry()
  World.entry();
  Vision.entry();
--Team and Gamecontrol are moved to main process
--  Team.entry();
--  GameControl.entry();
end

function update()
  count = count + 1;
  tstart = unix.time();

  -- update vision 
  imageProcessed = Vision.update();

  World.update_odometry();

  -- update localization
  if imageProcessed then
    nProcessedImages = nProcessedImages + 1;
    World.update_vision();

    if (nProcessedImages % 50 == 0) then
      if not webots then
        print('fps: '..(50 / (unix.time() - tUpdate)));
        tUpdate = unix.time();
      end
    end
  end
end

-- exit 
function exit()
--  GameControl.exit();
  Vision.exit();
--  Team.exit();
  World.exit();
end

