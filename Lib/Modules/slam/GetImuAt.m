function [vals tvals] = GetImuAt(t)
global IMU

vals = [];
tvals= [];

%wait for some data
if (IMU.cntr < IMU.histLen)
  fprintf(1,'GetImuAt: waiting for more imu data\n');
  return;
end

%make sure that the timestamps make sense
if (t>IMU.histTs(1) + 0.1) || (t<IMU.histTs(end))
  fprintf(1,'warning: no imu match could be found : %f %f %f\n', ...
             mod(t,1.0),mod(IMU.histTs(1),1.0),mod(IMU.histTs(end),1.0));
  return;
end


%find the match
for ii=1:IMU.histLen
  t2 = IMU.histTs(ii);
  dt = t-t2;
  if ( dt > 0)
    break;
  end
end

if (ii > 5)
  fprintf('GetImuAt: seems like a large delay : %f\n',dt);
end

vals = IMU.histVals{ii};
vals.roll  = vals.roll + vals.wroll*dt;
vals.pitch = vals.pitch + vals.wpitch*dt;
tvals = t2;