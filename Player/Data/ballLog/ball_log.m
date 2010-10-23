function ball_log(no)

%close all;
txt_str = sprintf('balllog%gm.txt',no);
fid = fopen(txt_str);
A = textscan(fid, '%f');
fclose(fid);

n = 9;

% File format : time, posx, posy, inst.velx, inst.vely, avg. velx, avg.vely
% kalman velx, kalman vely, kalman posx, kalman posy, predicted posx,
% predicted posy;

B = cell2mat(A);
B = reshape(B,n,length(B)/n);

t = B(1,:);
x = B(2,:);
y = B(3,:);
kvx = B(6,:);
kvy = B(7,:);
ckvx = B(8,:);
ckvy = B(9,:);

ivx = (x(2:end) - x(1:end-1)) ./ (t(2:end) - t(1:end-1));
ivy = (y(2:end) - y(1:end-1)) ./ (t(2:end) - t(1:end-1));
ivx = [0 ivx];
ivy = [0 ivy];

subplot(4,1,1),
hold on 
plot(x,'r');
plot(y,'g');
legend('X','Y');
hold off

subplot(4,1,2),
hold on
plot(ivx,'r');
plot(ivy,'g');
legend('Instantaneous X','Instantaneous Y');
hold off

subplot(4,1,3),
hold on 
plot(kvx,'b');
plot(kvy,'k');
legend('RKalman X','RKalman Y');
hold off

subplot(4,1,4),
hold on 
plot(ckvx,'b');
plot(ckvy,'k');
legend('CKalman X','CKalman Y');
hold off