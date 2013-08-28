%% Human Robot Interaction Test script
%% Copyright 2013 Stephen McGill

% Clear each time before execution
clear all;
% Add the path elements to get the mex stuff
addpath(genpath('.'))

warning off

global BODY

feedback_fd = udp_recv('new',54329);
feedback_s = zmq('fd',feedback_fd);

%% Setup the figure
f = figure(1);
clf;

BODY = robotbody();
BODY.init(gca);

drawnow;

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
    if s_idx == feedback_s
      BODY.update(feedback_fd);
      drawnow;
    end
  end
end

