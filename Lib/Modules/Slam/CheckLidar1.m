%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check lidar1 status
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ret = CheckLidar1()
global LIDAR1

ret=0;
if isempty(LIDAR1), return, end
if ~isfield(LIDAR1,'scan'), return, end
if ~isfield(LIDAR1.scan,'startTime'), return, end
%{
if (LIDAR1.scan.startTime - GetUnixTime() > LIDAR1.timeout)
  ret=0;
  return;
end
%}
ret=1;