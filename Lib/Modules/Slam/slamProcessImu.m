function slamProcessImu(data,name)
global IMU IMU_TS

if isempty(IMU_TS)
  IMU_TS.ts  = zeros(1,1000);
  IMU_TS.dts = zeros(1,1000);
  IMU_TS.cntr = 1;
end

IMU.data = MagicImuFilteredSerializer('deserialize',data);
IMU.histVals = [{IMU.data} IMU.histVals(1:end-1)];
IMU.histTs   = [IMU.data.t IMU.histTs(:,1:end-1)];
IMU.cntr = IMU.cntr + 1;

tnow = GetUnixTime();

%{
if (IMU_TS.cntr > 1)
  ti   = IMU.data.t;
  dtt= tnow-ti;
  tmod = mod(IMU_TS.cntr-1,1000)+1;
  IMU_TS.ts(tmod) = ti;
  IMU_TS.dts(tmod) = dtt; %ti-IMU_TS.ts(IMU_TS.cntr-1);%dtt;
end
IMU_TS.cntr = IMU_TS.cntr + 1;
%}

if (mod(IMU.cntr,100) == 0)
  dt = tnow - IMU.rateTime;
  fprintf('imu rate = %f\n',100/dt);
  IMU.rateTime = tnow;
end

dtImu = tnow - IMU.tLastArrival;
IMU.tLastArrival = tnow;