clear all;

%% List the logs
root = '~/Dropbox/IROS2016/data';
take = 'take3';
lsPlan = dir(fullfile(root, take, 'plan*.arm'));
lsAdlib = dir(fullfile(root, take, 'adlib*.arm'));
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
alpha1 = zeros(ranges(end), 1);
alpha2 = zeros(ranges(end), 1);
for i=2:numel(ranges)-1
    dr = [ranges(i)+1:ranges(i+1)] - ranges(i);
    alpha1(ranges(i)+1:ranges(i+1)) = exp(-(dr-1));
end

for i=2:numel(ranges)-1
    dr = [ranges(i-1)+1:ranges(i)] - ranges(i-1);
    alpha2(ranges(i-1)+1:ranges(i)) = -1*flip(exp(-(dr-1)));
end

alpha = alpha1+alpha2;

%% Identify the features
np = size(apath, 1);
% Similar config... (from Config_Arm)
qSimilar = deg2rad([-20, -60, -90, -120, 0, -45, 0]);
f1 = sum(apath - repmat(qSimilar, np, 1), 2).^2;

% Shoulder Yaw
f2 = apath(:, 2).^2;

% f3 is the mid of rnage of motion
qMid = [0, 0.759218, 0, -1.39626, 0, 0, 0];

f3 = apath - repmat(qMid, np, 1);
f3 = sum(f3, 2).^2;

% Elbow extension
f4 = apath(:, 4).^2;

% All features merged
features = [f1, f2, f3];

%% Weight the features
af = features .* repmat(alpha, [1, size(features, 2)]);
af = sum(af, 1);

%% Plot path
hPath = figure(1);
set(hPath, 'Position', [0, 0, 1024, 768]);
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

print(fullfile(root, 'adapted-path'),'-dpng');

%% Alpha weights
hAlpha = figure(2);
set(hAlpha, 'Position', [0, 0, 640, 480]);
plot(alpha);
for i=2:numel(ranges)-1
    line([ranges(i)+0.5, ranges(i)+0.5], [-pi, pi], 'LineWidth', 2, 'Color', 'k');
end

xlim([1, ranges(end)]);
ylim([-1, 1]);
title('Gradient Weighting', 'FontSize', 18);
xlabel('Timestep', 'FontSize', 16);
ylabel('Alpha', 'FontSize', 16);

print(fullfile(root, 'alpha-path'),'-dpng');