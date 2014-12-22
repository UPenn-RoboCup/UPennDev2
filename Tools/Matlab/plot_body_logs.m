joint = 'PelvL';
joint_idx = 1;
for i=1:numel(jointNames)
    str = jointNames{i};
    if strncmp(str, joint, 8)==1
        joint_idx = i;
    end
end

figure(1);
plot( ts(ts>50), rad2deg(pos(ts>50, joint_idx)), ...
    ts(ts>50), rad2deg(cmd(ts>50, joint_idx)) ...
    );
legend('Position', 'Command');
title('PelvL Command vs. Position');

hip_roll_pos = rad2deg(pos(:, joint_idx));
hip_roll_cmd = rad2deg(cmd(:, joint_idx));

joint = 'FootL';
joint_idx = 1;
for i=1:numel(jointNames)
    str = jointNames{i};
    if strncmp(str, joint, 8)==1
        joint_idx = i;
    end
end

foot_roll_pos = rad2deg(pos(:, joint_idx));
foot_roll_cmd = rad2deg(cmd(:, joint_idx));

% figure(2);
% plot( ts, rad2deg(pos(:, joint_idx)), ...
%     ts, rad2deg(cmd(:, joint_idx)) ...
%     );
% legend('Position', 'Command');
% title('FootL Command vs. Position (Degrees)');
% 
% figure(3);
% plot( ts, foot_roll_pos + hip_roll_pos, ...
%     ts, foot_roll_cmd + hip_roll_cmd, ...
%     ts, rad2deg(rpy(:, 1)) ...
%     );
% legend('Position', 'Command', 'IMU Roll');
% title('Combined Command vs. Position (Degrees)');
% 
% figure(4);
% plot( ts, cur(:, joint_idx) );
% title('Current (Amperes)');

% figure(5);
% plot(gyro(400:600,:));
% title('Gyro');
% legend('Roll', 'Pitch', 'Yaw');
% xlim([1 201]);
% % 
% figure(6);
% plot(rpy(400:600,1));
% title('Angle');
% legend('Roll', 'Pitch', 'Yaw');
% xlim([1 201]);

% figure(4);
% plot( ts, rad2deg(rpy(:, 1)) );
% title('Roll Angle (Degrees)');
% ylim([-5 5]);

figure(2);
plot(ts(ts>50), ft_l(ts>50,3));
title('Left Force Torque (Fz)');

figure(3);
plot(ts(ts>50), ft_r(ts>50,3));
title('Right Force Torque (Fz)');