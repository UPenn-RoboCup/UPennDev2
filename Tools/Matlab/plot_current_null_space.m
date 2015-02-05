close all;
% Call Replay Body logs first
%% Which time points
t0 = 1;
tf = 25;
t_selected = ts>t0&ts<tf;
t = ts(t_selected);

%% Get the figure handles
f_cmd_pos = figure;
h_n = subplot(2,1,1);
h_s = subplot(2,1,2);

f_left = figure;
h_yyL = gca;

f_right = figure;
h_yyR = gca;

%% Show the Left arm free parameter shoulder yaw joint
joint = 'LeftShoulderYaw';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

plot(h_n, t, rad2deg(pos(t_selected, joint_idx)), t, rad2deg(cmd(t_selected, joint_idx)) );
xlim(h_n, [t0, tf]);
legend(h_n, 'Position', 'Command');
title(h_n, sprintf('%s: Command vs. Position', joint));
xlabel(h_n,'Time (sec) with P Gain: 32');
ylabel(h_n,'Angle (deg)');

% Show current and position together
hAxL = plotyy(h_yyL, t, cur(t_selected, joint_idx), t, rad2deg(pos(t_selected, joint_idx)));
title(h_yyL, sprintf('%s: Current vs Position', joint));
xlabel(h_yyL,'Time (sec) with P Gain: 32');
ylabel(hAxL(1),'Current (mA)') % left y-axis
ylabel(hAxL(2),'Position (deg)') % right y-axis

%% Show the Right arm free parameter shoulder yaw joint
joint = 'RightShoulderYaw';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

plot(h_s, t, rad2deg(pos(t_selected, joint_idx)), t, rad2deg(cmd(t_selected, joint_idx)) );
xlim(h_s, [t0, tf]);
legend(h_s, 'Position', 'Command');
title(h_s, sprintf('%s: Command vs. Position', joint));
xlabel(h_s,'Time (sec) with P Gain: 32');
ylabel(h_s,'Angle (deg)');

% Show current and position together
hAxR = plotyy(h_yyR, t, cur(t_selected, joint_idx), t, rad2deg(pos(t_selected, joint_idx)));
title(h_yyR, sprintf('%s: Current vs Position', joint));
xlabel(h_yyR,'Time (sec) with P Gain: 32');
ylabel(hAxR(1),'Current (mA)') % left y-axis
ylabel(hAxR(2),'Position (deg)') % right y-axis

%% Draw and save
drawnow;
savefig(f_cmd_pos, 'Plots/nullSpace_LR_cmdVSpos_deg');
savefig(f_left, 'Plots/nullSpace_L_curVSpos_deg');
savefig(f_right, 'Plots/nullSpace_R_curVSpos_deg');

% Clear handles
clear h_nw h_sw h_ne h_se h_yyL h_yyR hAxL hAxR;
clear f_cmd_pos f_left f_right;