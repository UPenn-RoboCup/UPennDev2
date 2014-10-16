joint = 'LegLowerL';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

%{
figure;
subplot(2,1,1);
plot( ts, rad2deg(pos(:, joint_idx)), ...
    ts, rad2deg(cmd(:, joint_idx)) ...
    );
legend('Position', 'Command');
title(sprintf('%s: Command vs. Position (Degrees)', joint));

subplot(2,1,2);
plot( ts, cur(:, joint_idx) );
title(sprintf('%s: Current (milliAmperes)', joint));
%}

figure;
subplot(2,2,1);
plot( ts, rad2deg(pos(:, joint_idx)), ...
    ts, rad2deg(cmd(:, joint_idx)) ...
    );
legend('Position', 'Command');
title(sprintf('%s: Command vs. Position (Degrees)', joint));

subplot(2,2,3);
plot( ts, cur(:, joint_idx) );
title(sprintf('%s: Current (milliAmperes)', joint));




joint = 'LegLowerR';
joint_idx = 1;
for i=1:numel(jointNames)
    if strcmp(jointNames{i}, joint)==1
        joint_idx = i;
    end
end

subplot(2,2,2);
plot( ts, rad2deg(pos(:, joint_idx)), ...
    ts, rad2deg(cmd(:, joint_idx)) ...
    );
legend('Position', 'Command');
title(sprintf('%s: Command vs. Position (Degrees)', joint));

subplot(2,2,4);
plot( ts, cur(:, joint_idx) );
title(sprintf('%s: Current (milliAmperes)', joint));