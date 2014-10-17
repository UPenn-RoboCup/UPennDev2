%% Which time points
t0 = 5;
tf = 8;
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

%% Show the left leg
joint = 'LegLowerL';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end
plot( h_nw, t, rad2deg(pos(t_selected, joint_idx)), ...
    t, rad2deg(cmd(t_selected, joint_idx)) ...
    );
xlim(h_nw, [t0, tf]);
legend(h_nw, 'Position', 'Command');
title(h_nw, sprintf('%s: Command vs. Position (Degrees)', joint));

plot( h_sw, t, cur(t_selected, joint_idx) );
xlim(h_sw, [t0, tf]);
title(h_sw, sprintf('%s: Current (milliAmperes)', joint));

% Show current and position together
[hAx,hLine1,hLine2] = plotyy(h_yy, t, cur(t_selected, joint_idx),t,rad2deg(pos(t_selected, joint_idx)));
title(h_yy, sprintf('%s: Current (milliAmperes)', joint));
ylabel(hAx(1),'Slow Decay') % left y-axis
ylabel(hAx(2),'Fast Decay') % right y-axis

%% Show the right leg
joint = 'LegLowerR';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

plot( h_ne, t, rad2deg(pos(t_selected, joint_idx)), ...
    t, rad2deg(cmd(t_selected, joint_idx)) ...
    );
xlim(h_ne,[t0, tf]);
legend(h_ne,'Position', 'Command');
title(h_ne,sprintf('%s: Command vs. Position (Degrees)', joint));

plot( h_se, t, cur(t_selected, joint_idx) );
xlim(h_se,[t0, tf]);
title(h_se,sprintf('%s: Current (milliAmperes)', joint));

%% Clear the temp variables
drawnow;
clear h_nw h_sw h_ne h_se h_yy hAx hLine1 hLine2;