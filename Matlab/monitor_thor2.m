%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UPenn THOR Human-Robot Interface
%% Copyright 2013 Stephen McGill
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Clear each time before execution
clear all;
warning off;
close all;
addpath(genpath('.'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set up the event triggers
% Robot data

PORT_FEEDBACK  = 54329;
PORT_HCAMERA   = 33333;
PORT_MESH      = 33344;
PORT_LCAMERA   = 33335;
PORT_RCAMERA   = 33336;
PORT_SLAMMAP   = 22222;
PORT_HEIGHTMAP = 22223;

%Data sent to robots
PORT_RELIABLE_RPC = 55555; %for UI events
PORT_UNRELIABLE_RPC = 55556; %for UI events

WIRED_IP     = '192.168.123.26';
WIRELESS_IP  = '192.168.1.26';
LOCALHOST    = '127.0.0.1';
%ROBOT_IP     = WIRED_IP;
ROBOT_IP     = LOCALHOST;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup the figure
global POSE CAMERA LIDAR BODY SLAM NETMON CONTROL HMAP
setup_gui2();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Open the UDP Receivers
head_jimg_fd  = udp_recv('new', PORT_HCAMERA);
mesh_fd = udp_recv('new', PORT_MESH);
feedback_fd = udp_recv('new', PORT_FEEDBACK);

%% UDP/ZMQ Polling
s_head_jimg = zmq( 'fd', head_jimg_fd );
s_mesh = zmq( 'fd', mesh_fd );
s_feedback = zmq( 'fd', feedback_fd );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set up the network callbacks
% +1 since the socket handles begin at 0
% TODO: maybe chagne this in the mex file?
s_callbacks = {};
s_callbacks{s_head_jimg+1} = CAMERA.update_head;
s_callbacks{s_feedback+1} = BODY.update;
s_callbacks{s_mesh+1} = LIDAR.update;

% Monitor these callbacks in the network monitor
NETMON.num_callbacks = numel(s_callbacks);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set up callback names
global CALLBACK_NAMES;
CALLBACK_NAMES = {};
for i=1:numel(s_callbacks)
    callback_names{i} = 'Unknown';
end
callback_names{s_head_jimg+1} = 'Head Camera';
callback_names{s_feedback+1} = 'Feedback';
callback_names{s_mesh+1} = 'Mesh Image';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sending commands to the robot
CONTROL.udp_send_id = udp_send( 'init', ROBOT_IP, PORT_UNRELIABLE_RPC );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Timing
t_init = tic;
t0 = tic;
target_fps = 30;
inv_target_fps = 1/target_fps;
dirty = 0;
counter = 0;
drawnow;

netmon_count = 0;
netmon_interval = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Begin the main loop
while 1
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Poll and perform callbacks
    % 1 FPS timeout
    idx = zmq('poll',1000*inv_target_fps);
    for s=1:numel(idx)
        s_idx = idx(s);
        [s_data,has_more] = zmq( 'receive', s_idx );
        % +1 since the socket handles begin at 0, but MATLAB indexes at 1
        s_idx_m = s_idx + 1;
        s_callback  = s_callbacks{s_idx_m};
        nBytes = s_callback(s_data);
        NETMON.update(s_idx_m,nBytes);
        % We have new data to draw
        dirty = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control the drawing update interval
    t_diff = toc(t0);
    counter = counter+1;
    if t_diff>inv_target_fps
        % Update the title
        netmon_count =netmon_count+1;
        if mod(netmon_count,netmon_interval)==0
            NETMON.redraw(1/t_diff,POSE.battery);
        end
        % Update timing
        counter = 0;
        t0 = tic;
        if dirty==1
            dirty = 0;
        end
    end
    drawnow;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end % main loop