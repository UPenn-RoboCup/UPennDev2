%close all;

%% Which time points
t0 = 5;
tf = 10;
t_selected = ts>t0&ts<tf;
t = ts(t_selected);

%% Get the figure handles
figure;
h_nw = subplot(2,2,1);
h_sw = subplot(2,2,3);
h_ne = subplot(2,2,2);
h_se = subplot(2,2,4);

figure;
h_yy = gca;

%% Show the left leg pitch
joint = 'ShoulderR';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

lleg = pos(t_selected, joint_idx:joint_idx+5);
z_lleg = zeros(size(lleg,1), 1);
for i=1:numel(z_lleg)
    tr = kinematics(lleg(i,:));
    z_lleg(i) = tr(3,4);
end

plot( h_nw, t, pos(t_selected, joint_idx), t, cmd(t_selected, joint_idx) );
xlim(h_nw, [t0, tf]);
legend(h_nw, 'Position', 'Command');
title(h_ne,'Left Pitch Command vs. Position (Degrees)');
%title(h_nw, sprintf('%s: Command vs. Position (Degrees)', joint));

plot( h_sw, t, cur(t_selected, joint_idx) );
xlim(h_sw, [t0, tf]);
title(h_sw, sprintf('%s: Current (milliAmperes)', joint));

% Show current and position together
hAx = plotyy(h_yy, t, sum(abs(cur(t_selected, joint_idx+2:joint_idx+4)),2), t, z_lleg);
title(h_yy, sprintf('%s: Current (milliAmperes)', joint));
ylabel(hAx(1),'Current (mA)') % left y-axis
ylabel(hAx(2),'Position (Degrees)') % right y-axis

%% Show the right leg pitch
joint = 'LegUpperR';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

rleg = pos(t_selected, joint_idx) + pos(t_selected, joint_idx+1) + pos(t_selected, joint_idx+2);
rleg_cmd = cmd(t_selected, joint_idx) + cmd(t_selected, joint_idx+1) + cmd(t_selected, joint_idx+2);

plot( h_ne, t, rad2deg(rleg), ...
    t, rad2deg(rleg_cmd) ...
    );
xlim(h_ne,[t0, tf]);
legend(h_ne,'Position', 'Command');
title(h_ne,'Right Pitch Command vs. Position (Degrees)');
%title(h_ne,sprintf('%s: Command vs. Position (Degrees)', joint));

plot( h_se, t, cur(t_selected, joint_idx) );
xlim(h_se,[t0, tf]);
title(h_se,sprintf('%s: Current (milliAmperes)', joint));

%% Clear the temp variables
drawnow;
clear h_nw h_sw h_ne h_se h_yy hAx;