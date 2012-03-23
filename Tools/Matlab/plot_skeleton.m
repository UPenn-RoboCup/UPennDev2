%% Plot the skeleton
%clear all;
if( exist('sk','var') == 0 )
    sk = shm_primesense();
end

run_once = 0;

jointNames = { ...
    'Waist', 'Torso', 'Head', 'Neck', ...
    'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL', ...
    'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR', ...
    'HipL', 'KneeL', 'AnkleL', 'FootL', ...
    'HipR', 'KneeR', 'AnkleR', 'FootR'...
    };

joint2track = 'ShoulderL';
index2track = find(ismember(jointNames, joint2track)==1);

nJoints = numel(jointNames);
positions = zeros(nJoints,3);
rots = zeros(3,3,nJoints);
confs = zeros(nJoints,2);
axis_angles = zeros(nJoints,4);
axis_angles_loc = zeros(nJoints,4);
scale = 10;

figure(1);clf;
while(1)
    t0=tic;
    % Loop
    for j=1:nJoints
        jName = jointNames{j};
        joint = sk.get_joint( jName );
        positions(j,:) = joint.position;
        rots(:,:,j) = joint.rot;
        axis_angles(j,:) = vrrotmat2vec(joint.rot);
        confs(j,:) = joint.confidence;
    end
    [ local_rots ] = abs2local_rot( rots );
    for j=1:nJoints
        axis_angles_loc(j,:) = vrrotmat2vec(local_rots(:,:,j));
    end
    positions = positions - repmat(positions(1,:),nJoints,1); % Center at waist
    %axis_angles2 = axis_angles(:,1:3) .* repmat(axis_angles(:,4),1,3);
    axis_angles2 = axis_angles_loc(:,1:3) .* repmat(axis_angles_loc(:,4),1,3) .* repmat(confs(:,2)~=0,1,3);
    clf;
    plot3( positions(:,1), positions(:,2), positions(:,3), 'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor',[.49 1 .63], 'MarkerSize',10 );
    hold on;
    quiver3(positions(:,1), positions(:,2), positions(:,3), axis_angles2(:,1),axis_angles2(:,2),axis_angles2(:,3),'b-','LineWidth',3 );
    view(0,90);
    axis([-1000 1000 -1300 700 -1000 1000]);
    %plot( positions(:,1), positions(:,2), 'o', ...
    %    'MarkerEdgeColor','k', 'MarkerFaceColor',[.49 1 .63], 'MarkerSize',10 );
    
%{
    [x,y] = pol2cart(axis_angles(index2track,4),1);
    axis_tmp = axis_angles(index2track,:);
    subplot(1,2,1);
    quiver(0,0,x,y,'r-', 'LineWidth',5);
    axis([-1 1 -1 1]);
    subplot(1,2,2);
    quiver3(0,0,0, axis_tmp(1),axis_tmp(2),axis_tmp(3),'b-','LineWidth',5 );
    axis([-1 1 -1 1 -1 1]);
    %}
    %{
    if( confs(index2track,2)~=0 )
        rotplot( reshape(rots(index2track,:,:),[3 3]), [1;1;1] );
    else
        fprintf('Not confident about %s rotation\n',joint2track);
    end
    %}

    %{
    clf;
    for j=5:8
        hold on;
        rotplot( rots(:,:,j), positions(j,:)', scale );
    end
    drawnow;
    %}
    
    if( run_once==1 )
        return;
    end
    
    % Run at 30Hz
    %pause(0.033);
    % Run at 10Hz
    %pause(0.1);
    toc(t0);
    % Run at 5Hz
    pause(0.2);
    
end