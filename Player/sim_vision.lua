cwd = os.getenv('PWD');

require('init')
require('Config')
Config.dev.camera = 'SimCam';
Config.dev.body = 'SimBody';

require('unix')
require('vcm')
require('getch')
require ('Broadcast')
require('Vision')

Vision.entry();

getch.enableblock(1);

count = 0;
tUpdate = unix.time();

Broadcast.update(2);
Broadcast.update_img (2);

while (true) do
  count = count + 1;
  tstart = unix.time();

  -- update vision 
  imageProcessed = Vision.update();
  if (imageProcessed) then
    print ('Image Processed!')
  end
  unix.sleep(1.0);
end

-- exit 
Vision.exit();

