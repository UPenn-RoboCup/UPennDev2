cwd = os.getenv('PWD')
require('init')

require('Hokuyo')
require('signal')
require('unix')
require('rcm')

function ShutDownFN()
  print("Proper shutdown")
  Hokuyo.shutdown()
  os.exit(1);
end

hokuyo = {}
hokuyo.serial = "00805676"
--hokuyo.serial = "00907258"
--hokuyo.device = "/dev/ttyACM1"
hokuyo.device = "/dev/ttyACM0"
Hokuyo.open(hokuyo.device, hokuyo.serial);

cntr = 0;
t0 = unix.time();
while (true) do
  Hokuyo.update();
  cntr = cntr + 1;
  if (cntr % 40 == 0) then
    print("Scan rate "..40/(unix.time() - t0));
    t0 = unix.time();
  end

  lidar = Hokuyo.retrieve();
  
  rcm.set_lidar_counter(lidar.counter);
  rcm.set_lidar_id(lidar.id);
  rcm.set_lidar_ranges(lidar.ranges);
  rcm.set_lidar_startAngle(lidar.startAngle);
  rcm.set_lidar_stopAngle(lidar.stopAngle);
  rcm.set_lidar_startTime(lidar.startTime);
  rcm.set_lidar_startTime(lidar.startTime);
  

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
