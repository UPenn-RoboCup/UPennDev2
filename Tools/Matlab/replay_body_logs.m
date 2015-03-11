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

%% Aquire the body joint angles
timestamp = '12.31.2008.19.17.41';
% TODO: Check if already available
%load(strcat('Data/joint_m_',timestamp,'.mat'));

fid = fopen(strcat('Data/joint_m_',timestamp,'.log'));
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
cur = zeros(numel(jobjs), numel(jobjs{1}.i));
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
    cur(i,:) = jobj.i;
    ft_l(i,:) = jobj.ft_l;
    ft_r(i,:) = jobj.ft_r;
    gyro(i,:) = jobj.gyro;
    acc(i,:) = jobj.acc;
    rpy(i,:) = jobj.rpy;
end
clear jobj
%% Save
clear jobjs;save(strcat('Data/joint_m_',timestamp,'.mat'));