clear all;
%% Joint angle helpers
jointNames = { ...
	'Neck','Head', ...
	'ShoulderL', 'ArmUpperL', 'LeftShoulderYaw', ...
	'ArmLowerL','LeftWristYaw','LeftWristRoll','LeftWristYaw2', ... 
	'PelvYL','PelvL','LegUpperL','LegLowerL','AnkleL','FootL',...
	'PelvYR','PelvR','LegUpperR','LegLowerR','AnkleR','FootR',...
	'ShoulderR', 'ArmUpperR', 'RightShoulderYaw','ArmLowerR',...
	'RightWristYaw','RightWristRoll','RightWristYaw2',...
	'TorsoPitch','TorsoYaw',...
	'l_wrist_grip1','l_wrist_grip2','l_wrist_grip3',...
	'r_wrist_grip1','r_wrist_grip2','r_wrist_grip3',...
	'ChestLidarPan',...
};

joint = 'AnkleL';
joint_idx = 1;
for i=1:numel(jointNames)
    str = jointNames{i};
    if strncmp(str, joint, 4)==1
        joint_idx = i;
    end
end
clear str;

%% Aquire the body joint angles
fid = fopen('Logs/joint_m_06.04.2014.13.54.20.log');
msg = fread(fid,inf,'*uchar');
fclose(fid);
clear fid;
jobjs = msgpack('unpacker', msg);
clear msg;

%% Time
t0 = jobjs{1}.t;

%% Reformat the data
ts  = zeros(numel(jobjs), 1);
pos = zeros(numel(jobjs), numel(jobjs{1}.p));
cmd = zeros(numel(jobjs), numel(jobjs{1}.cp));
ft_l = zeros(numel(jobjs), numel(jobjs{1}.ft_l));
ft_r = zeros(numel(jobjs), numel(jobjs{1}.ft_r));
gyro = zeros(numel(jobjs), numel(jobjs{1}.gyro));
acc = zeros(numel(jobjs), numel(jobjs{1}.acc));
rpy = zeros(numel(jobjs), numel(jobjs{1}.rpy));
for i=1:numel(jobjs)
    jobj = jobjs{i};
    ts(i)    = jobj.t - t0;
    pos(i,:) = jobj.p;
    cmd(i,:) = jobj.cp;
    ft_l(i,:) = jobj.ft_l;
    ft_r(i,:) = jobj.ft_r;
    gyro(i,:) = jobj.gyro;
    acc(i,:) = jobj.acc;
    rpy(i,:) = jobj.rpy;
end
clear jobjs jobj
%% Save
save('Logs/joint_06.04.2014.13.54.20.mat')
%% Plot a joint
figure(1);
plot( ts, pos(:, joint_idx), ...
    ts, cmd(:, joint_idx) ...
    );
legend('Position', 'Command');
title('Command vs. Position');

figure(2);
plot(ft_l);
title('Left Force Torque');

figure(3);
plot(ft_r);
title('Right Force Torque');

figure(4);
plot(gyro);
title('Gyro');

figure(5);
plot(rpy);
title('RPY');