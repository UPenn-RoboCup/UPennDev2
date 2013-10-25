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
idx = find(strncmp(joint,jointNames,4));

%% Aquire the body joint angles
%fid = fopen('logs/body_10.24.2013.20.28.32_meta.log');
fid = fopen('logs/body_10.24.2013.20.31.04_meta.log');
msg = fread(fid,inf,'*uchar');
fclose(fid);
jobjs = msgpack('unpacker', msg);
clear msg;

%% Reformat the data
pos = zeros(numel(jobjs),numel(jobjs{1}.position) );
cmd = zeros(numel(jobjs),numel(jobjs{1}.command_position) );
ts  = zeros(numel(jobjs),1);
for i=1:numel(jobjs)
    jobj = jobjs{i};
    ts(i)    = jobjs{i}.t;
    pos(i,:) = jobjs{i}.position;
    cmd(i,:) = jobjs{i}.command_position;
end

%% Plot a joint
figure(1);
clf;
plot( ts, pos(:,idx), 'b' );
hold on;
plot( ts, cmd(:,idx), 'r' );
hold off;