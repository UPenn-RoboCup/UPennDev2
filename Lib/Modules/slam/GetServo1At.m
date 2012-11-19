function [val tval] = GetServo1At(t)
global SERVO1

val  = [];
tval = [];

%check the counter
if (SERVO1.cntr < SERVO1.histLen)
  fprintf('GetServoAt: waiting for more servo data\n');
  return;
end

%make sure that the timestamps make sense
if (t>SERVO1.histTs(1) + 0.15) || (t<SERVO1.histTs(end))
  fprintf(1,'GetServoAt: warning: no servo match could be found : %f %f %f\n', ...
             mod(t,1.0),mod(SERVO1.histTs(1),1.0),mod(SERVO1.histTs(end),1.0));
  return;
end

%find the match
for ii=1:SERVO1.histLen
  t2 = SERVO1.histTs(ii);
  dt = t-t2;
  if ( dt > 0)
    break;
  end
end

if (ii > 5)
  fprintf('GetServo1At: seems like a large delay: %f\n',dt);
end


val = SERVO1.histVals(1,ii);
%fprintf('dt = %f\n',dt);

if (ii > 1)           %interpolate
  da  = SERVO1.histVals(1,ii-1) - SERVO1.histVals(1,ii);
  dta = SERVO1.histTs(ii-1) - SERVO1.histTs(ii);
  
  dadt = da / dta;
  
  if (dta > 0.015)   %make sure that the data didn't come at the same time
    val = val + dadt * dt;
  end
else                  %extrapolate
  da  = SERVO1.histVals(1,1) - SERVO1.histVals(1,2);
  dta = SERVO1.histTs(1) - SERVO1.histTs(2);
  
  dadt = da / dta;
  
  if (dta > 0.015)   %make sure that the data didn't come at the same time
    val = val + dadt * dt;
  end
end

tval = t2;


