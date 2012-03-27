%% Plot the skeleton
%clear all;
if( exist('sk','var') == 0 )
    startup;
    sk = shm_primesense();
end

%% Timing settings
prep_time = 10;
nseconds_to_log = 5;
run_once = 0;
counter = 0;
fps = 20;
twait = 1/fps;
logsz = nseconds_to_log * fps;

%% Joint Settings
jointNames = { ...
    'Waist', 'Torso', 'Head', 'Neck', ...
    'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL', ...
    'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR', ...
    'HipL', 'KneeL', 'AnkleL', 'FootL', ...
    'HipR', 'KneeR', 'AnkleR', 'FootR'...
    };

joint2track = 'ShoulderL';
index2track = find(ismember(jointNames, joint2track)==1);

%% Initialize variables
nJoints = numel(jointNames);
positions = zeros(nJoints,3);
rots = zeros(3,3,nJoints);
confs = zeros(nJoints,2);
% Logging variables
jointLog(logsz).t = 0;
jointLog(logsz).positions = positions;
jointLog(logsz).rots = rots;
jointLog(logsz).confs = confs;


%% 5 second prep
for i=prep_time:-1:1
    disp(i);
    pause(1);
end

%% Figure
figure(1);clf;
p=plot( positions(:,1), positions(:,2), 'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor', [.49 1 .63], 'MarkerSize',10 );
    axis([-1000 1000 -1300 700]);
    
%% Go time
t0=tic;
t_passed=toc(t0);
while(t_passed<nseconds_to_log)
    tstart=tic;
    counter = counter + 1;
    %% Loop through each joint
    for j=1:nJoints
        jName = jointNames{j};
        joint = sk.get_joint( jName );
        positions(j,:) = joint.position;
        rots(:,:,j) = joint.rot;
        confs(j,:) = joint.confidence;
    end
    t_passed=toc(t0);
    positions = positions - repmat(positions(1,:),nJoints,1); % Center at waist
    
    %% Plot the data
    set(p, 'XData', positions(:,1));
    set(p, 'YData', positions(:,2));
    
    %% Append Log
    jointLog(counter).t = t_passed;
    jointLog(counter).positions = positions;
    jointLog(counter).rots = rots;
    jointLog(counter).confs = confs;
    
    %% Timing
    if( run_once==1 )
        break;
    end
    
    tf = toc(tstart);
    pause( max(twait-tf,0) );
    
end

%% Save data
save(strcat('primeLogs_',datestr(now,30)),'jointNames','jointLog')