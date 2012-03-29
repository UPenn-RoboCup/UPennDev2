%% Plot the skeleton
%clear all;

load('primeLogs_20120328T235032.mat');
debug = 0;

joint2track = 'ElbowL';
index2track = find(ismember(jointNames, joint2track)==1);
indexWaist = find(ismember(jointNames, 'Waist')==1);

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
    rots = jointLog(i).rots;
    confs = jointLog(i).confs;
    axis_angles_loc = zeros(nJoints,4);
    [ local_rots ] = abs2local_rot( rots );
    for j=1:nJoints
        axis_angles_loc(j,:) = vrrotmat2vec(local_rots(:,:,j));
        rpy_loc(j,:) = dcm2angle( local_rots(:,:,j) ) * 180/pi;
    end
    
    % Only if we have confidence...
    axis_angles_mag = axis_angles_loc(:,4);
    axis_angles_loc = axis_angles_loc(:,1:3) .* ...
        repmat(axis_angles_loc(:,4),1,3);
    %% Update Figure
    % Update plot3
    pc = confs(:,1)>0;
    rc = confs(:,2)>0;
    ci = center_idx & pc;
    li = left_idx & pc;
    ri = right_idx & pc;
    figure(1);
    clf;
    p_left=plot3( positions(li,1), positions(li,2), positions(li,3),'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor', 'r', 'MarkerSize',10 );
    hold on;
    p_right=plot3( positions(ri,1), positions(ri,2), positions(ri,3),'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor', 'g', 'MarkerSize',10 );
    p_center=plot3( positions(ci,1), positions(ci,2), positions(ci,3),'o', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor', 'b', 'MarkerSize',10 );
    q = quiver3(positions(pc&rc,1), positions(pc&rc,2), positions(pc&rc,3), ...
        axis_angles_loc(pc&rc,2),axis_angles_loc(pc&rc,1),axis_angles_loc(pc&rc,3), ...
        'b-','LineWidth',3 );
    % Front view
    view(0,90);
    % Side view
    %view(-90,0);
    % Top View
    %view(0,0);
    axis([-1000 1000 -1200 1500 -1000 1000]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    %% Print Debug data for Joint2Track
    if( debug==1 )
        fprintf('%s: (Pos: %.3f,Rot: %.3f)\n', ...
            joint2track, confs(index2track,1), confs(index2track,1) );
        fprintf('Roll: %.3f, Pitch: %.3f, Yaw: %.3f\n', ...
            rpy_loc(index2track,3),rpy_loc(index2track,2),rpy_loc(index2track,1));
        fprintf('Position: %.3f, %.3f, %.3f\n\n',...
            positions(index2track,1),positions(index2track,2),positions(index2track,3) );
        disp(axis_angles_mag(index2track));
    end
    tf = toc(tstart);
    % Realistic pause
    pause( max(twait-tf,0) );
    
    % Arbitrary pause:
    pause(.5);
end

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
        rotplot( rots(:,:,j), positions(j,:)', 10 );
    end
    drawnow;
%}