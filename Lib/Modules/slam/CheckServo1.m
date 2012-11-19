function ret = CheckServo1()
global SERVO1

ret=0;
if isempty(SERVO1), return, end
if ~isfield(SERVO1,'data'), return, end
if ~isfield(SERVO1.data,'t'), return, end

if isempty(SERVO1.tLastArrival);
   fprintf(1,'waiting for initial servo message\n');
end

%check if data is old
currTime = GetUnixTime();
dt = currTime - SERVO1.tLastArrival;
if ( dt > SERVO1.timeout)
  fprintf(1,'servo data is old!! : dt = %f\n',dt);
  return;
end



ret=1;
