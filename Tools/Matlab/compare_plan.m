clear all;

%% List the logs
lsPlan = dir('~/Dropbox/IROS2016/data/take2/plan*.arm');
lsAdlib = dir('~/Dropbox/IROS2016/data/take2/adlib*.arm');
paths = cell(1, numel(lsPlan));
breaks = [];

%% Grab the path
for i=1:numel(lsPlan)
    disp(i);
    %% Grab the generated path
    fid = fopen(fullfile(lsPlan(i).folder, lsPlan(i).name));
    raw = fread(fid,Inf,'*uint8');
    fclose(fid);
    clear fid;
    obj = msgpack('unpack', raw);
    clear raw;
    nq = numel(obj.lpath{1});
    np = numel(obj.lpath);
    paths{i} = reshape(cell2mat(obj.lpath), [nq, np])';
    clear obj;
end

%% Grab the adlib on the path
for i=1:numel(lsAdlib)
    disp(i);
    fid = fopen(fullfile(lsAdlib(i).folder, lsAdlib(i).name));
    raw = fread(fid,Inf,'*uint8');
    fclose(fid);
    clear fid;
    if numel(raw)==0
        break;
    end
    objs = msgpack('unpacker', raw);
    clear raw;
    qStop = objs{1}.qLArm0;
    qDesired = objs{end}.qAdlib;
    clear objs;
    path = paths{i};
    % Find the breakpoint
    for j=1:size(path, 1)
        dnorm = norm(path(j,:)-qStop);
        fprintf('Break: %d %d/%d %.6f\n', i, j, size(path, 1), dnorm);
        if dnorm < 1e-6
            breaks(i) = j;
            break
        end
    end
    
end

%% Split up
fpaths = cell(size(paths));
offsets = zeros(1, numel(paths));
for i=1:numel(paths)
    if i<=numel(breaks)
        fpaths{i} = paths{i}(1:breaks(i), :);
    else
        fpaths{i} = paths{i};
    end
    offsets(i) = size(fpaths{i}, 1);
end
ranges = [0, cumsum(offsets)];

%% Join together as one
apath = zeros(ranges(end), nq);
for i=1:numel(fpaths)
    apath(ranges(i)+1:ranges(i+1), :) = fpaths{i};
end

%% Add the alpha weighting
alpha1 = zeros(1, ranges(end));
alpha2 = zeros(1, ranges(end));
for i=2:numel(ranges)-1
    dr = [ranges(i)+1:ranges(i+1)] - ranges(i);
    alpha1(ranges(i)+1:ranges(i+1)) = exp(-dr);
end

for i=2:numel(ranges)-1
    dr = [ranges(i-1)+1:ranges(i)] - ranges(i-1);
    alpha2(ranges(i-1)+1:ranges(i)) = -1*flip(exp(-dr));
end
figure(2);
plot(alpha1+alpha2);

%% Plot
figure(1);
clf;
hold on;
for i=1:numel(fpaths)
    plot(ranges(i)+1:ranges(i+1), fpaths{i});
end
hold off;

for i=2:numel(ranges)-1
    line([ranges(i)+0.5, ranges(i)+0.5], [-pi, pi], 'LineWidth', 2, 'Color', 'k');
end
xlim([1, ranges(end)]);
ylim([-pi, pi]);
title('Adapted Path', 'FontSize', 18);
xlabel('Timestep', 'FontSize', 16);
ylabel('Angle (rad)', 'FontSize', 16);
h_legend = legend(...
    'Sh Pitch',...
    'Sh Yaw',...
    'Sh Roll',...
    'Elbow',...
    'Wr Roll 1',...
    'Wr Yaw',...
    'Wr Roll 2');
h_legend.FontSize = 16;
h_legend.Location = 'best';
