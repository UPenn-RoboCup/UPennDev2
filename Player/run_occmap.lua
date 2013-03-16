cwd = os.getenv('PWD')
require('init')

require('Config')
require('OccupancyMap')

maxFPS = 15;
tperiod = 1.0/maxFPS;
maxDisFPS = 5;
DisCount = 0;

OccupancyMap.entry();

while (true) do
  tstart = unix.time();

  OccupancyMap.update();

  tloop = unix.time() - tstart;

  DisCount = DisCount + 1
  if (DisCount % maxDisFPS == 0) then
    print('OccMap Update Time: '..tloop);
  end
  if (tloop < tperiod) then
    unix.usleep((tperiod - tloop)*(1E6));
  end
end

OccupancyMap.exit();

