%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check lidar0 status
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ret = CheckLidar0()
global LIDAR0

ret=0;
if isempty(LIDAR0), return, end
if ~isfield(LIDAR0,'scan'), return, end
if ~isfield(LIDAR0.scan,'startTime'), return, end
%{
if (LIDAR0.scan.startTime - GetUnixTime() > LIDAR0.timeout)
  ret=0;
  return;
end
%}
ret=1;