clear variables;
USE_TASK = 0;
USE_PLOT = 0;

%% List the logs
root = '~/Dropbox/IROS2016/data';
take = 'take5';
lsPlan = dir(fullfile(root, take, 'plan*.arm'));
lsAdlib = dir(fullfile(root, take, 'adlib*.arm'));
lsJs = dir(fullfile(root, take, 'js*.arm'));
paths = cell(1, numel(lsPlan));
breaks = [];

%% Grab the path
for i=1:numel(lsPlan)
    % Grab the generated path
    fid = fopen(fullfile(lsPlan(i).folder, lsPlan(i).name));
    raw = fread(fid,Inf,'*uint8');
    fclose(fid);
    obj = msgpack('unpack', raw);
    nq = numel(obj.lpath{1});
    np = numel(obj.lpath);
    paths{i} = reshape(cell2mat(obj.lpath), [nq, np])';
    if i==1
        plan0 = obj;
        plan0.lpath = paths{i};
    end
end
clear fid obj raw;

%% Grab the adlib on the path
for i=1:numel(lsAdlib)
    fid = fopen(fullfile(lsAdlib(i).folder, lsAdlib(i).name));
    raw = fread(fid,Inf,'*uint8');
    fclose(fid);
    
    if numel(raw)==0
        break;
    end
    objs = msgpack('unpacker', raw);
    
    qStop = objs{1}.qLArm0;
    qDesired = objs{end}.qAdlib;
    
    path = paths{i};
    % Find the breakpoint
    for j=1:size(path, 1)
        dnorm = norm(path(j,:)-qStop);
        %fprintf('Break: %d %d/%d %.6f\n', i, j, size(path, 1), dnorm);
        if dnorm < 1e-6
            breaks(i) = j;
            break
        end
    end
    
end
clear fid obj raw;

%% Grab the Jacobians
for i=1:numel(lsJs)
    load(fullfile(lsJs(i).folder, lsJs(i).name), '-mat');
    if i==1
        plan0.Js = Js;
        plan0.nulls = nulls;
    end
end
clear Js nulls;

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
qGravity = deg2rad([0, 60, 90, -120, -90, -15, 0]);
f1 = sum(apath - repmat(qGravity, np, 1), 2).^2;

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
clear f1 f2 f3;

%% Weight the features
af = features .* repmat(alpha, [1, size(features, 2)]);
af = sum(af, 1);

fprintf('%s | Weighted features: %.2f %.2f %.2f\n', take, af);

%% Loss of the path
% Pure loss of the path - how well it actually followed, by the magnitude
% of the human interaction, essentially
losses = zeros(numel(ranges)-2, nq);
normLosses = zeros(size(losses, 1), 1);
for i=2:numel(ranges)-1
    before = apath(ranges(i), :); % bad by robot
    after = apath(ranges(i)+1, :); % Good by user
    losses(i-1,:) = before - after;
    normLosses(i-1) = norm(losses(i-1,:));
end
fprintf('%s | Loss of the path: %.2f\n', take, sum(normLosses));


%% FK readings
if USE_TASK==1
    msg = msgpack('pack', apath);
    fid = fopen('/tmp/qp.path', 'w');
    fwrite(fid, msg);
    fclose(fid);
    clear fid msg;
    % RUN LUA SCRIPT
    fid = fopen('/tmp/fk.mp');
    raw = fread(fid,Inf,'*uint8');
    fclose(fid);
    clear fid;
    obj = msgpack('unpack', raw);
    clear raw;
    fkPath = [obj{:}];
    fkPath = reshape(fkPath, [6, numel(fkPath)/6])';
end

%% Re-run the path optimization
optimize_augmented(plan0.lpath, plan0.Js, plan0.nulls);

%% Plot things
if USE_PLOT
    
    %% Plot path
    hPath = figure(1);
    clf;
    set(hPath, 'Position', [0, 0, 1024, 768]);
    hold on;
    for i=1:numel(fpaths)
        plot(ranges(i)+1:ranges(i+1), rad2deg(fpaths{i}));
    end
    hold off;
    
    for i=2:numel(ranges)-1
        line([ranges(i)+0.5, ranges(i)+0.5], [-180, 180], 'LineWidth', 2, 'Color', 'k');
    end
    xlim([1, ranges(end)]);
    ylim([-180, 180]);
    title('Adapted Path', 'FontSize', 18);
    xlabel('Timestep', 'FontSize', 16);
    ylabel('Angle (deg)', 'FontSize', 16);
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
    
    %% Alpha weights
    hAlpha = figure(2);
    clf;
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
    
    %% Task Space plot
    if USE_TASK==1
        hTask = figure(3);
        clf;
        set(hTask, 'Position', [0, 0, 640, 480]);
        x = 1:size(fkPath,1);
        [hAx, h1, h2] = plotyy(...
            x, fkPath(:, 1:3), ...
            x, rad2deg(fkPath(:, 4:6)) );
        for i=2:numel(ranges)-1
            line([ranges(i)+0.5, ranges(i)+0.5], [-pi, pi], 'LineWidth', 2, 'Color', 'k');
        end
        xlim(hAx(1), [1, ranges(end)]);
        xlim(hAx(2), [1, ranges(end)]);
        title('Task Space Path', 'FontSize', 18);
        xlabel('Timestep', 'FontSize', 16);
        ylabel(hAx(1), 'Position (m)', 'FontSize', 16);
        ylabel(hAx(2), 'Angle (deg)', 'FontSize', 16);
        h_legend = legend(hAx(1),...
            'x',...
            'y',...
            'z');
        h_legend.FontSize = 16;
        h_legend.Location = 'best';
        h_legend = legend(hAx(2),...
            'Roll',...
            'Pitch',...
            'Yaw');
        h_legend.FontSize = 16;
        h_legend.Location = 'best';
    end
    
end
%% Save the images
%print(fullfile(root, 'adapted-path'),'-dpng');
%print(fullfile(root, 'alpha-path'),'-dpng');
%print(fullfile(root, 'adapted-task'),'-dpng');