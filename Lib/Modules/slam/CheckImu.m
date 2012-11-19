function ret = CheckImu()
global IMU

ret = 0;
if isempty(IMU),return, end
if ~isfield(IMU,'data')
  fprintf(1,'waiting for initial imu message\n');
  return;
end
if ~isfield(IMU.data,'t')
  fprintf(1,'no t field in the imu data struct\n');
  return
end

if isempty(IMU.tLastArrival)
  fprintf(1,'waiting for initial imu message\n');
  return;
end

%check if data is old
currTime = GetUnixTime();
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
