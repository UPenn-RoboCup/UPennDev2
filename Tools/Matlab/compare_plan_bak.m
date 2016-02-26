clear all;

lpaths = {};
breaks = [];

%% Grab the generated path
fid = fopen('~/Dropbox/IROS2016/data/plan_1456425385_1.arm');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
obj = msgpack('unpack', raw);
clear raw;
nq = numel(obj.lpath{1});
np = numel(obj.lpath);
lpath0 = cell2mat(obj.lpath);
lpaths{1} = reshape(lpath0, [nq, np])';
clear obj;

%% Grab the adlib on the path
fid = fopen('~/Dropbox/IROS2016/data/adlib_1456425385_1.arm');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
objs = msgpack('unpacker', raw);
clear raw;
qStop = objs{1}.qLArm0;
qDesired = objs{end}.qAdlib;
clear objs;
% Find the breakpoint
for i=1:size(lpaths{1}, 1)
    dnorm = norm(lpaths{1}(i,:)-qStop);
    if dnorm < 1e-6
        breaks(1) = i;
        break
    end
end

%% Grab the generated path
fid = fopen('~/Dropbox/IROS2016/data/plan_1456425394_2.arm');
raw = fread(fid, Inf, '*uint8');
fclose(fid);
clear fid;
obj = msgpack('unpack', raw);
clear raw;
nq = numel(obj.lpath{1});
np = numel(obj.lpath);
lpath0 = cell2mat(obj.lpath);
lpaths{2} = reshape(lpath0, [nq, np])';
clear obj;

%% Plot
figure(1);
clf;
plot(1:breaks(1), lpaths{1}(1:breaks(1),:));
hold on;
plot(breaks(1)+1:breaks(1)+size(lpaths{2}, 1), lpaths{2});
hold off;
for i=1,numel(breaks)
    line([breaks(i), breaks(i)], [-pi, pi], 'LineWidth', 2, 'Color', 'k');
end
title('Path 1');
ylim([-pi, pi]);

% figure(2);
% clf;
% plot(lpaths{2});
% title('Path 2');
