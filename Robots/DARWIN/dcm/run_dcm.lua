cwd = os.getenv('PWD')
require('init')

require('unix');
require('shm');
require('dcm');

print('Starting device comm manager...');
opcm = require('OPCommManager');
opcm.entry()

print('Running controller');
loop = true;
count = 0;
t0 = unix.time();

fpsdesired=100; --100 HZ cap on refresh rate
ncount=200;

dcm.set_actuator_read_enable(1) 
dcm.set_actuator_torqueEnable(0)
dcm.set_actuator_torqueEnableChanged(1)


opcm.update()

local torque_enabled=false


t_timing=unix.time();





while (loop) do
   count = count + 1;
   local t1 = unix.time();
   local tPassed=math.max(math.min(t1-t_timing,0.010),0); --Check for timer overflow
   if dcm.get_actuator_read_enable()==1 then ncount=2 else ncount=200 end

--[[
dcm.set_sensor_gyro({0,0,count})
print(unpack(dcm.get_sensor_gyro() ))
local gyro_ptr = dcm.sensorPtr.gyro
print(gyro_ptr[0],gyro_ptr[1],gyro_ptr[2])
print("----")
--]]

   t_timing=t1;
   opcm.update()
   if (count % ncount == 0) then
      if not torque_enabled then
--
	dcm.set_actuator_read_enable(0) 
	dcm.set_actuator_torqueEnable(1)
	dcm.set_actuator_torqueEnableChanged(1)
	dcm.set_actuator_hardnessChanged(1)
	dcm.set_actuator_gainChanged(1)

	torque_enabled=true
	print("TORQUE_ENABLED!!")
--]]
      end
      os.execute("clear")
      print(
	string.format("IMU Acc: %.2f %.2f %.2f ",unpack(dcm.get_sensor_accelerometer()   ))..
	string.format("Gyr: %.1f %.1f %.1f ",unpack(dcm.get_sensor_gyro())) ..
	string.format("Angle: %.1f %.1f %.1f ",unpack(dcm.get_sensor_rpy() ))..
	string.format("/ %d FPS", ncount/(t1-t0) )
	)
      t0 = t1;



      print(string.format("C_Position:\n Head: %.1f %.1f\n Larm: %.1f %.1f %.1f\n Lleg: %.1f %.1f %.1f %.1f %.1f %.1f\n Rleg: %.1f %.1f %.1f %.1f %.1f %.1f\n Rarm: %.1f %.1f %.1f\n",
			  unpack(vector.new(dcm.get_actuator_command_position())*180/math.pi)
		    ));


      print(string.format("A_Position:\n Head: %.1f %.1f\n Larm: %.1f %.1f %.1f\n Lleg: %.1f %.1f %.1f %.1f %.1f %.1f\n Rleg: %.1f %.1f %.1f %.1f %.1f %.1f\n Rarm: %.1f %.1f %.1f\n",
			  unpack(vector.new(dcm.get_sensor_position())*180/math.pi)
		    ));

   end
end

opcm.exit()
