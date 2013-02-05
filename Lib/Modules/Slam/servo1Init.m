function servo1Init
global SERVO1

if isempty(SERVO1) || ~isfield(SERVO1,'initialized') ||(SERVO1.initialized ~= 1)
  SERVO1.msgName = [GetRobotName '/Servo1'];
  SERVO1.data    = [];
  SERVO1.timeout = 0.1;
  SERVO1.hist    = [];
  SERVO1.tLastArrival = [];
  SERVO1.cntr    = 0;
  SERVO1.rateTime = GetUnixTime();
  
  SERVO1.histLen  = 10;
  SERVO1.histVals = zeros(1,SERVO1.histLen);
  SERVO1.histTs   = zeros(1,SERVO1.histLen);
  
  
  SERVO1.initialized = 1;
  
  ipcAPIDefine(SERVO1.msgName,MagicServoStateSerializer('getFormat'));
  disp('Servo1 initialized');
end
  
