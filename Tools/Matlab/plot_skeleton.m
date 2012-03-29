%% Plot the skeleton
%clear all;

load('primeLogs_20120328T235032.mat');
debug = 0;

joint2track = 'ElbowL';
index2track = find(ismember(jointNames, joint2track)==1);

%% For quick calculations
%const double upperArmLength = .060;  //OP, spec
%const double lowerArmLength = .129;  //OP, spec
op_arm_len = .189;
indexWaist = find(ismember(jointNames, 'Waist')==1);
indexShoulder = find(ismember(jointNames, 'ShoulderL')==1);
indexElbow = find(ismember(jointNames, 'ElbowL')==1);
indexWrist = find(ismember(jointNames, 'WristL')==1);
indexFoot = find(ismember(jointNames, 'FootL')==1);

nLogs = numel(jointLog);
nJoints = numel(jointNames);
positions = jointLog(1).positions;
rots = jointLog(1).rots;
confs = jointLog(1).confs;
axis_angles_loc = zeros(nJoints,4);
rpy_loc = zeros(nJoints,3);
[ local_rots ] = abs2local_rot( rots );
for j=1:nJoints
    axis_angles_loc(j,:) = vrrotmat2vec(local_rots(:,:,j));
end
pc = confs(:,1)>0;
rc = confs(:,2)>0;
ci = center_idx & pc;
li = left_idx & pc;
ri = right_idx & pc;
f = figure(1);
clf;
p_left=plot3( positions(li,1), positions(li,2), positions(li,3),'o', ...
    'MarkerEdgeColor','k', 'MarkerFaceColor', 'r', 'MarkerSize',10 );
hold on;
p_right=plot3( positions(ri,1), positions(ri,2), positions(ri,3),'o', ...
    'MarkerEdgeColor','k', 'MarkerFaceColor', 'g', 'MarkerSize',10 );
p_center=plot3( positions(ci,1), positions(ci,2), positions(ci,3),'o', ...
    'MarkerEdgeColor','k', 'MarkerFaceColor', 'b', 'MarkerSize',10 );

q=quiver3(positions(:,1), positions(:,2), positions(:,3), ...
    axis_angles_loc(:,1),axis_angles_loc(:,2),axis_angles_loc(:,3), ...
    'b-','LineWidth',3 );
    
% Front view
view(0,90);
% Side view
%view(-90,0);
% Top View
%view(0,0);
axis([-1 1 -1.2 1.5 -1 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
for i=1:nLogs-1
    tstart=tic;
    
    % Check data limits
    if( isempty(jointLog(i).t) || i>80 )
        break;
    end
    if( isempty(jointLog(i+1).t) )
        twait = 0;
    else
        twait = jointLog(i+1).t - jointLog(i).t;
    end
    %% Get data
    positions = jointLog(i).positions - repmat(jointLog(i).positions(indexWaist,:), nJoints,1);
    positions = positions / 1000;
    rots = jointLog(i).rots;
    confs = jointLog(i).confs;
    axis_angles_loc = zeros(nJoints,4);
    [ local_rots ] = abs2local_rot( rots );
    for j=1:nJoints
        axis_angles_loc(j,:) = vrrotmat2vec(local_rots(:,:,j));
        rpy_loc(j,:) = dcm2angle( local_rots(:,:,j) ) * 180/pi;
    end
    
    %% Arm calculations
    e2w = positions(indexElbow,:) - positions(indexWrist,:);
    s2e = positions(indexShoulder,:) - positions(indexElbow,:);
    s2w = positions(indexShoulder,:) - positions(indexWrist,:);
    arm_len = sqrt(norm(e2w)) + sqrt(norm(s2e));
    offset = s2w * (op_arm_len / arm_len);
    
    % Only if we have confidence...
    axis_angles_mag = axis_angles_loc(:,4);
    axis_angles_loc = axis_angles_loc(:,1:3) .* ...
        repmat(axis_angles_loc(:,4),1,3) .* repmat(confs(:,2)~=0,1,3);
    %% Update Figure
    % Update plot3
    pc = confs(:,1)>0;
    rc = confs(:,2)>0;
    ci = center_idx & pc;
    li = left_idx & pc;
    ri = right_idx & pc;
    set(p_left, 'XData', positions( li, 1), ...
        'YData', positions( li, 2), ...
        'ZData', positions( li, 3) ...
        );
    set(p_right, 'XData', positions( ri, 1), ...
        'YData', positions( ri, 2), ...
        'ZData', positions( ri, 3));
    set(p_center, 'XData', positions( ci, 1), ...
        'YData', positions( ci, 2), ...
        'ZData', positions( ci, 3));
    % Update quiver
    axis_angles_loc = axis_angles_loc .* repmat(pc&rc,1,3);
    set(q, 'XData', positions(:,1), ...
        'YData', positions(:,2), ...
        'ZData', positions(:,3), ...
        'UData', axis_angles_loc(:,1), ...
        'VData', axis_angles_loc(:,2), ...
        'WData', axis_angles_loc(:,3) ...
        );
    
    
    %% Timing
    tf = toc(tstart);
    % Realistic pause
    pause( max(twait-tf,0) );
    % Arbitrary pause:
    %pause(.2);
end