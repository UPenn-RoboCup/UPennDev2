module(... or "", package.seeall)

require('camnition')
require('getch')
getch.enableblock(1);

maxFPS = Config.vision.maxFPS;
tperiod = 1.0/maxFPS;
broadcast_enable=0;

camnition.entry();

while (true) do
  tstart = unix.time();

  -- Get key press
  local str=getch.get();
  if #str>0 then
    local byte=string.byte(str,1);
    if byte==string.byte("g") then  --Broadcast selection
      local mymod = 4;
      broadcast_enable = (broadcast_enable+1)%mymod;
      print("Broadcast:", broadcast_enable);
    end
  end
  vcm.set_camera_broadcast(broadcast_enable);
  
  camnition.update();

  tloop = unix.time() - tstart;

  if (tloop < tperiod) then
    unix.usleep((tperiod - tloop)*(1E6));
  end
end

camnition.exit();
