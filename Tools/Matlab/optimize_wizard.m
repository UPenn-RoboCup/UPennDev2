% Boyd & Vandenberghe, "Convex Optimization"
% Joëlle Skaf - 08/23/05
%
% Solved a QCQP with 3 inequalities:
%           minimize    1/2 x'*P0*x + q0'*r + r0
%               s.t.    1/2 x'*Pi*x + qi'*r + ri <= 0   for i=1,2,3
% and verifies that strong duality holds.

%% Initialization
% Listen for requests
clear all;
ch = zmq('reply', 'ipc', 'armopt');

%% Main loop
while 1
    p = zmq('poll', 1000);
    if numel(p)>0
        %% Load new plan
        tmpfile = zmq('receive', ch);
        tmpfile = char(tmpfile);
        %packed_msg = zmq('receive', ch);
        %msg = msgpack('unpack', packed_msg);
        fprintf(1, 'Optimizing %s\n', tmpfile);
        % Input data
        load(char(tmpfile));
        
        % Run the optimizer
        optimize_armplan;
        
        % Save results to be opened
        save(tmpfile, 'q');
        y = zmq('send', ch, tmpfile);
        
        fprintf(1,'Done! \n');
        % Show
        show_armplan;

    %else disp('Timeout...');
    end
    
end