%% Plot the skeleton
clear all;
sk = shm_primesense();

jointNames = { 'Head', 'Neck', 'Torso', 'Waist', ...
    'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL', ...
    'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR', ...
    'HipL', 'KneeL', 'AnkleL', 'FootL', ...
    'HipR', 'KneeR', 'AnkleR', 'FootR'...
    };
nJoints = numel(jointNames);
positions = zeros(nJoints,3);

figure(1);
while(1)
    
    % Loop
    for j=1:nJoints
        jName = jointNames{j};
        joint = sk.get_joint( jName );
        positions(j,:) = joint.position;
    end
    plot3( positions(:,1), positions(:,2), positions(:,3), 'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor',[.49 1 .63], 'MarkerSize',10 );
    
    % Run at 30Hz
    pause(0.033);
    
end