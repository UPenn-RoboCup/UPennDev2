clear all;

lpaths = {};

fid = fopen('/tmp/arm_plan_1455216464_1.plan');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
obj = msgpack('unpack', raw);
nq = numel(obj.lpath{1});
np = numel(obj.lpath);
lpath0 = cell2mat(obj.lpath);
lpaths{1} = reshape(lpath0, [nq, np])';
clear obj;

fid = fopen('/tmp/arm_plan_1455216491_28.plan');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
obj = msgpack('unpack', raw);
nq = numel(obj.lpath{1});
np = numel(obj.lpath);
lpath0 = cell2mat(obj.lpath);
lpaths{2} = reshape(lpath0, [nq, np])';
clear obj;

figure(1);
clf;
plot(lpaths{1});

figure(2);
clf;
plot(lpaths{2});

%% TODO: Find the index of the transition
