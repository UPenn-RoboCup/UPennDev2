require('unix');
require('shm');

local cwd = unix.getcwd();
package.path = cwd.."/../Util/?.lua;"..package.path; --For Transform
package.path = cwd.."/../Vision/?.lua;"..package.path; --For vcm

dcm = require('CharliCommManager');
print('Starting device comm manager...');
dcm.entry()

-- I don't think this should be here for shm management?
-- Shouldn't these just be dcm.something acces funcitons?
sensorShm = shm.open('dcmSensor');
actuatorShm = shm.open('dcmActuator');

require('vcm') --Shared memory is created here, and ready for access

local imu = require('microstrain')
imu.open('/dev/ttyACM0')
imu.set_continuous(0)



print('Running controller');
loop = true;
count = 0;
t0 = unix.time();
    

--for testing only
dcm.actuator.readType[1]=1; --Read ALL servos
dcm.actuator.battTest[1]=1; --Battery test enable

fpsdesired=100; --100 HZ cap on refresh rate
ncount=2;

t_timing=unix.time();
while (loop) do
   
   count = count + 1;
   local t1 = unix.time();
   local tPassed=math.max(math.min(t1-t_timing,0.010),0); --Check for timer overflow
   t_timing=t1;
   readtype= actuatorShm:get('readType') ;
   if readtype==0 then ncount=20;
     else ncount = 5;
   end 

   dcm.update()
   -- Update imu
   imu.request_data()
   imu_data = imu.receive_data()
   --XYZ in g unit
   acc=vector.new({-imu_data[2],-imu_data[1],-imu_data[3]})/9.8;
   --RPY in rad/s unit
   gyr=vector.new({imu_data[5],imu_data[4],-imu_data[6]});
   --RPY in rad unit
   angle=vector.new({imu_data[8],imu_data[7],imu_data[9]});

   if (count % ncount == 0) then
      os.execute("clear");
      print(
	string.format("IMU Acc: %.2f %.2f %.2f ",unpack(acc))..
	string.format("Gyr: %.1f %.1f %.1f ",unpack(gyr*180/math.pi))..
	string.format("Angle: %.1f %.1f %.1f ",unpack(angle*180/math.pi))..
	string.format("/ %d FPS [%d]", ncount/(t1-t0),readtype)
	)
      t0 = t1;

      print(string.format("Button: %d %d",  unpack(sensorShm:get('button'))));      

      pos = vector.new(sensorShm:get('position'))*180/math.pi;
      servopos = vector.new(sensorShm:get('servoposition'));
      bias = vector.new(sensorShm:get('bias'));
      temp = vector.new(sensorShm:get('temperature'));

      print(
	string.format("Position:\n")..
	string.format(
		"head:%.1f %.1f\n"..
		"LArm:%.1f %.1f %.1f %.1f\n"..
		"LLeg:%.1f %.1f %.1f %.1f %.1f %.1f\n"..
		"RLeg:%.1f %.1f %.1f %.1f %.1f %.1f\n"..
		"RArm:%.1f %.1f %.1f %.1f\n"..
--		"Waist:%.1f\nHands:%.1f %.1f\n",
		"Waist:%.1f\n",
		unpack(pos)
		)
	);
      print(
	string.format("Servo Position:\n")..
	string.format(
		"head:%d %d\n"..
		"LArm:%d %d %d %d\n"..
		"LLeg:%d %d %d %d %d %d\n"..
		"RLeg:%d %d %d %d %d %d\n"..
		"RArm:%d %d %d %d\n"..
--		"Waist:%d\n"..
--		"Hands:%d %d\n",
		"Waist:%d\n",
		unpack(servopos)
		)
	);

      print(string.format("Battery: %.1f V\n", sensorShm:get('battery')/10));

      print(
	string.format("Servo Temp:\n")..
	string.format(
		"head:%.1f %.1f\n"..
		"LArm:%.1f %.1f %.1f %.1f\n"..
		"LLeg:%.1f %.1f %.1f %.1f %.1f %.1f\n"..
		"RLeg:%.1f %.1f %.1f %.1f %.1f %.1f\n"..
		"RArm:%.1f %.1f %.1f %.1f\n"..
--		"Waist:%.1f\n"..
--		"Hands:%.1f %.1f\n",
		"Waist:%.1f\n",
		unpack(temp)
		)
	);

   end
end

dcm.exit()
imu.close()
