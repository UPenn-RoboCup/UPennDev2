%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Servo1 message handler 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamProcessServo1(data,name)
global SERVO1

SERVO1.data     = MagicServoStateSerializer('deserialize',data);
SERVO1.histVals = [double(SERVO1.data.position) SERVO1.histVals(:,1:end-1)];
SERVO1.histTs   = [SERVO1.data.t SERVO1.histTs(:,1:end-1)]; 

SERVO1.cntr = SERVO1.cntr + 1;

tnow = GetUnixTime();

if (mod(SERVO1.cntr,40) == 0)
  dt = tnow - SERVO1.rateTime;
  fprintf('servo rate = %f\n',40/dt);
  SERVO1.rateTime = tnow;
end

dtServo1 = tnow - SERVO1.tLastArrival;

SERVO1.tLastArrival = tnow;
