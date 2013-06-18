%% Human Robot Interaction Test script
%% Copyright 2013 Stephen McGill

% Clear each time before execution
clear all;
% Add the path elements to get the mex stuff
addpath(genpath('.'))
warning off
global LIDAR CHEST_LIDAR HEAD_LIDAR

range_fd = udp_recv('new',54345);
range_s = zmq('fd',range_fd);

%% Setup the figure
f = figure(1);
clf;

LIDAR = lidarbody();

%{
subplot(2,2,1);
a1=gca;
subplot(2,2,2);
a2=gca;
subplot(2,2,3);
a3=gca;
subplot(2,2,4);
a4=gca;
%}

a1=0;
a3=0;

subplot(1,2,1);
a2=gca;
subplot(1,2,2);
a4=gca;



LIDAR.init(a1,a2,a3,a4);

%% Timing
t0 = tic;
target_fps = 30;
inv_target_fps = 1/target_fps;
dirty = 0;
counter = 0;

%% Begin the main loop
while 1
  counter = counter+1;
  % 1 FPS timeout
  idx = zmq('poll',1000*inv_target_fps);
  for s=1:numel(idx)
    s_idx = idx(s);
    [s_data,has_more] = zmq( 'receive', s_idx );
    if s_idx == range_s
      LIDAR.update(range_fd);
      drawnow;
    end
  end
end

