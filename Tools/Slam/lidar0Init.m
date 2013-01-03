
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function lidar0Init
global LIDAR0

if isempty(LIDAR0) || ~isfield(LIDAR0,'initialized') || (LIDAR0.initialized ~= 1)
  LIDAR0.scan    = [];
  LIDAR0.timeout = 0.1;
  LIDAR0.lastTime = [];
  
  LIDAR0.initialized = 1;
  disp('Lidar0 initialized');
end
