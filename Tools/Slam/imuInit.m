
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function imuInit
global IMU

if isempty(IMU) || ~isfield(IMU,'initialized') || (IMU.initialized ~= 1)
  IMU.data    = [];
  
  IMU.histLen  = 20;
  IMU.histVals = cell(1,IMU.histLen);
  IMU.histTs   = zeros(1,IMU.histLen);
   
  IMU.initialized = 1;
  %IMU.initDelta   = []; %time between first imu packettimestamp and unix time at the moment of reception
  IMU.tLastArrival       = []; %time of arrival of last packet
  IMU.timeout = 0.1;
  IMU.cntr = 0;
  %IMU.rateTime = GetUnixTime();
  
  disp('Imu initialized');
end