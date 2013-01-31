
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function ret = CheckImu()
global IMU

ret = 0;
if isempty(IMU),return, end
if ~isfield(IMU,'data')
  fprintf(1,'waiting for initial imu message 1\n');
  return;
end
if ~isfield(IMU.data,'t')
  fprintf(1,'no t field in the imu data struct\n');
  return
end

if isempty(IMU.tLastArrival)
  fprintf(1,'waiting for initial imu message 2\n');
  return;
end

%check if data is old
currTime = 0;
dt = currTime - IMU.tLastArrival;
if ( dt > IMU.timeout)
  fprintf(1,'imu data is old!! : dt = %f\n',dt);
  return;
end

%check values
if ( (abs(IMU.data.roll) > pi/4) || (abs(IMU.data.pitch) > pi/4) )
  fprintf(1,'imu data is invalid: rpy = %f %f %f\n',IMU.data.roll,IMU.data.pitch,IMU.data.yaw);
  return;
end

ret=1;
