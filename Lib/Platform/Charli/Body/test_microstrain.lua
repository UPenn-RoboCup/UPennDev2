require('unix');
require('shm');

local cwd = unix.getcwd();
package.path = cwd.."/../Util/?.lua;"..package.path; --For Transform
package.path = cwd.."/../Vision/?.lua;"..package.path; --For vcm

require('vector')

--local log = io.open('Logs/pgain_02_squat_4s_imu.txt', 'w+')

local imu = require('microstrain')
imu.open('/dev/ttyACM0')
imu.set_continuous(0)

local t, t0, dt = unix.time(), unix.time(), 0.005

while true do
  -- Update time
  dt = unix.time() - t
  unix.usleep(math.max(1950 - 1e6*dt, 0))
  t = unix.time() 

  -- Update imu
  imu.request_data()
  imu_data = imu.receive_data()
  
  --XYZ in g unit
  acc=vector.new({-imu_data[2],-imu_data[1],-imu_data[3]})/9.8;
  --RPY in rad/s unit
  gyr=vector.new({imu_data[5],imu_data[4],-imu_data[6]});
  --RPY in rad unit
  angle=vector.new({imu_data[8],imu_data[7],imu_data[9]});
  os.execute("clear");
  print(string.format("Acc XYZ: %3.3f %3.3f %3.3f",unpack(acc) ));
  print(string.format("Gyr RPY: %3d %3d %3d",unpack(gyr*180/math.pi)  ));
  print(string.format("Angle RPY:%3d %3d %3d",unpack(angle*180/math.pi)  ));
end

imu.close()
--log:close()
