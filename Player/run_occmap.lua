module(... or "", package.seeall)

require('OccupancyMap')

maxFPS = 15;
tperiod = 1.0/maxFPS;

Occupancy.entry();

while (true) do
  tstart = unix.time();

  OccupancyMap.update();

  tloop = unix.time() - tstart;

  if (tloop < tperiod) then
    unix.usleep((tperiod - tloop)*(1E6));
  end
end

OccupancyMap.exit();

