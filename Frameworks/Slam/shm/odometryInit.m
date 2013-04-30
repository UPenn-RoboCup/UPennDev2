function odometryInit
global ODOMETRY

if isempty(ODOMETRY) || (ODOMETRY.initialized ~= 1)
  ODOMETRY.odom           = [];
  ODOMETRY.acounts          = zeros(4,1);
  ODOMETRY.tLastReset       = [];
  ODOMETRY.tLast            = [];
  ODOMETRY.cntr             = 0;
  
  ODOMETRY.initialized  = 1;
  disp('Encoders initialized');
end
