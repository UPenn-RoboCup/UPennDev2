%% Access to JPEG and msgpack mex files
addpath( genpath('.') );

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
idx = 1;
for i=1:numel(jointNames)
    str = jointNames{i};
    if strncmp(str, joint, 4)==1
        idx = i;
    end
end

%% Aquire the body joint angles
fid = fopen('Logs/joint_m_06.04.2014.13.54.20.log');
msg = fread(fid,inf,'*uchar');
fclose(fid);
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
    ts(i)    = jobjs{i}.t - t0;
    pos(i,:) = jobjs{i}.p;
    cmd(i,:) = jobjs{i}.cp;
    ft_l(i,:) = jobjs{i}.ft_l;
    ft_r(i,:) = jobjs{i}.ft_r;
    gyro(i,:) = jobjs{i}.gyro;
    acc(i,:) = jobjs{i}.acc;
    rpy(i,:) = jobjs{i}.rpy;
end

%% Plot a joint
figure(1);
h_p = plot( ts, pos(:, idx), ...
    ts, cmd(:, idx) ...
    );
h_l = legend(h_p, 'Position', 'Command');
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