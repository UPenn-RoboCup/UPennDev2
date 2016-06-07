jointNames = { ...
  'ShoulderL', 'ArmUpperL', 'LeftShoulderYaw', ...
  'ArmLowerL','LeftWristYaw','LeftWristRoll','LeftWristYaw2'...
};

 %% open other
%{
load ~/plan0.mat
vwp0 = [vwEffective{:}]';
vw0 = [vwPath{:}]';
qw0 = [qwPath{:}]';
 
load ~/planEnd.mat
vwp = [vwEffective{:}]';
vw = [vwPath{:}]';
qw = [qwPath{:}]';
 
save('~/Desktop/pref_vw.mat')
%}

%%
load ~/Desktop/pref_vw.mat
figure(1);
subplot(2,2,1);
plot(vwp0);
title('Preference Initial Rate');
xlabel('Timestep');
ylabel('Task Velocity');
xlim([1, size(vwp0, 1)]);
ylim([-0.4 0.4]);
rms_pref0 = rms(vwp0);
drate_pref0 = diff(vwp0);

subplot(2,2,2);
vw0 = abs(vw0);
plot(vw0);
title('Preference Initial Convergence');
xlabel('Timestep');
ylabel('Task Offset');
xlim([1, size(vw0, 1)]);
ylim([0, 3]);

subplot(2,2,3);
plot(vwp);
title('Preference Optimized Rate');
xlabel('Timestep');
ylabel('Task Velocity');
xlim([1, size(vwp, 1)]);
ylim([-0.4 0.4]);
rms_pref1 = rms(vwp);
drate_pref1 = diff(vwp);

subplot(2,2,4);
vw = abs(vw);
plot(vw);
title('Preference Optimized Convergence');
xlabel('Timestep');
ylabel('Task Offset');
xlim([1, size(vw, 1)]);
ylim([0, 3]);

%%
load ~/Desktop/rrts_vw.mat
figure(2);
subplot(2,2,1);
plot(vwp0);
title('RRT* Initial Rate');
xlabel('Timestep');
ylabel('Task Velocity');
xlim([1, size(vwp0, 1)]);
ylim([-0.4 0.4]);
rms_rrts0 = rms(vwp0);
drate_rrts0 = diff(vwp0);

subplot(2,2,2);
vw0 = abs(vw0);
plot(vw0);
title('RRT* Initial Convergence');
xlabel('Timestep');
ylabel('Task Offset');
xlim([1, size(vw0, 1)]);
ylim([0, 3]);

subplot(2,2,3);
plot(vwp);
title('RRT* Optimized Rate');
xlabel('Timestep');
ylabel('Task Velocity');
xlim([1, size(vwp, 1)]);
ylim([-0.4 0.4]);
rms_rrts1 = rms(vwp);
drate_rrts1 = diff(vwp);

subplot(2,2,4);
vw = abs(vw);
plot(vw);
title('RRT* Optimized Convergence');
xlabel('Timestep');
ylabel('Task Offset');
xlim([1, size(vw, 1)]);
ylim([0, 3]);

%% RMS
ymax = max([rms_pref0, rms_pref1, rms_rrts0, rms_rrts1]);

figure(3);
subplot(2,2,2);
bar(rms_pref0);
title('RMS Pref Original');
xlabel('Task Coordinate');
ylabel('Velocity (m/s, rad/s)');
ylim([0, ymax*1.2]);
xlim([0.5, 6.5]);

subplot(2,2,4);
bar(rms_pref1);
title('RMS Pref Optimized');
xlabel('Task Coordinate');
ylabel('Velocity (m/s, rad/s)');
ylim([0, ymax*1.2]);
xlim([0.5, 6.5]);

subplot(2,2,1);
bar(rms_rrts0);
title('RMS RRT* Original');
xlabel('Task Coordinate');
ylabel('Velocity (m/s, rad/s)');
ylim([0, ymax*1.2]);
xlim([0.5, 6.5]);

subplot(2,2,3);
bar(rms_rrts1);
ylim([0, ymax*1.2]);
xlim([0.5, 6.5]);
title('RMS RRT* Optimized');
xlabel('Task Coordinate');
ylabel('Velocity (m/s, rad/s)');

%{
hl = legend({...
    'x','y','z',...
    'roll','pitch','yaw'});
set(hl, 'FontSize', 18);
%}

%% Rate of change in the task space: should be constant, for predictability
%{
figure(4);

subplot(2,2,1);
plot(drate_pref0);
title('Rate Pref Original');
xlabel('Timestep');

subplot(2,2,3);
plot(drate_pref1);
title('Rate Pref Optimized');
xlabel('Timestep');

subplot(2,2,2);
plot(drate_rrts0);
title('Rate RRT* Original');
xlabel('Timestep');

subplot(2,2,4);
plot(drate_rrts1);
title('Rate RRT* Optimized');
xlabel('Timestep');
%}

%% Joint space smoothness
figure(5);

load ~/Desktop/pref_vw.mat

subplot(2,2,2);
plot(qw0);
xlim([1, size(qw, 1)]);
ylim([-pi, pi]);
title('Pref Joints Original');
xlabel('Timestep');
ylabel('Angle (rad)');

%hl = legend(jointNames);
%set(hl, 'FontSize', 18)

subplot(2,2,4);
plot(qw);
xlim([1, size(qw0, 1)]);
ylim([-pi, pi]);
title('Pref Joints Optimized');
xlabel('Timestep');
ylabel('Angle (rad)');

load ~/Desktop/rrts_vw.mat

subplot(2,2,1);
plot(qw0);
xlim([1, size(qw, 1)]);
ylim([-pi, pi]);
title('RRT* Joints Original');
xlabel('Timestep');
ylabel('Angle (rad)');

subplot(2,2,3);
plot(qw);
xlim([1, size(qw0, 1)]);
ylim([-pi, pi]);
title('RRT* Joints Optimized');
ylabel('Angle (rad)');
ylabel('Radians');
