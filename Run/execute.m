% function Rleg_U = execute(s,rpy)
% file = fopen(s,'r');
% file2 = fopen(rpy,'r');
% 
% data = fscanf(file,'%f');
% rpy  = fscanf(file2,'%f');
clearvars

data = double(load('com_v4g2.txt'));
data_stage = data(:,1);
idx = find(data_stage == 13);
%{
data_x = [data(1:idx(1)-1,2:4),data(1:idx(1)-1,8:10),data(1:idx(1)-1,14:23)];
data_y = [data(1:idx(1)-1,24:26),data(1:idx(1)-1,30:32),data(1:idx(1)-1,36:45)];
data_z = [data(1:idx(1)-1,46:48),data(1:idx(1)-1,52:54),data(1:idx(1)-1,58:67)];
torso = [data(1:idx(1)-1,22),data(1:idx(1)-1,44),data(1:idx(1)-1,66)];
%}
data_x = [data(:,2:4),data(:,8:10),data(:,14:23)];
data_y = [data(:,24:26),data(:,30:32),data(:,36:45)];
data_z = [data(:,46:48),data(:,52:54),data(:,58:67)];
torso = [data(:,22),data(:,44),data(:,66)];


[step_com, col_com] = size(data);

figure(1)
plot(data_z(:,15),'-o')



%{
for i=1:step_com
    figure(1);
    clf;
    for k=1:16
        plot3(data_x(i,k),data_y(i,k),data_z(i,k),'ko','MarkerFaceColor','k','MarkerSize',10);
        hold on
    end
    plot3(data_x(i,7:10),data_y(i,7:10),data_z(i,7:10),'r');
    hold on
    plot3(data_x(i,11:14),data_y(i,11:14),data_z(i,11:14),'b');
    hold on
    plot3(data_x(i,1:3),data_y(i,1:3),data_z(i,1:3),'c');
    hold on
    plot3(data_x(i,4:6),data_y(i,4:6),data_z(i,4:6),'y');
    hold on
    plot3(data_x(i,15:16),data_y(i,15:16),data_z(i,15:16),'g');
    hold on
    plot3([data_x(i,7),data_x(i,11)],[data_y(i,7),data_y(i,11)],[data_z(i,7),data_z(i,11)],'r');
    hold on
    plot3([data_x(i,16),data_x(i,1)],[data_y(i,16),data_y(i,1)],[data_z(i,16),data_z(i,1)],'r');
    hold on
    plot3([data_x(i,16),data_x(i,4)],[data_y(i,16),data_y(i,4)],[data_z(i,16),data_z(i,4)],'r');
    axis equal
    axis([-1,1,-1,1,-1,1]*0.8)
    pause(0.001)
end
%}
%{
Rleg_U= zeros(step_com, 4);
Rleg_L= zeros(step_com, 4);
RFoot= zeros(step_com, 4);
Lleg_U= zeros(step_com, 4);
Lleg_L= zeros(step_com, 4);
LFoot= zeros(step_com, 4);
Rarm_U= zeros(step_com, 4);
Rarm_AB= zeros(step_com, 4);
Rarm_L= zeros(step_com, 4);
Rarm_Wr= zeros(step_com, 4);
Larm_U= zeros(step_com, 4);
Larm_AB= zeros(step_com, 4);
Larm_L= zeros(step_com, 4);
Larm_Wr= zeros(step_com, 4);
Torso= zeros(step_com, 4);
Pelvis= zeros(step_com, 4);

com_xyz = [sum(data_x,2),sum(data_y,2),sum(data_z,2)];
figure(3)
subplot(3,1,1)
plot(com_xyz(:,1))
subplot(3,1,2)
plot(com_xyz(:,2))
subplot(3,1,3)
plot(com_xyz(:,3))

figure(4)
subplot(3,1,1)
plot(torso(:,1))
subplot(3,1,2)
plot(torso(:,2))
subplot(3,1,3)
plot(torso(:,3))
%}
