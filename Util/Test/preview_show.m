clear all;
fid = fopen('com_zmp.tmp');
d = fread(fid,inf,'double');
fclose(fid);

% Massage the data
n_data = 6;
com_zmp = reshape(d,n_data,numel(d)/n_data)';
clear d;

% Subsets
com_x = com_zmp(:,1);
com_y = com_zmp(:,2);
zmp_x = com_zmp(:,3);
zmp_y = com_zmp(:,4);
step_id = com_zmp(:,5);
t = com_zmp(:,6);
clear com_zmp;

fid = fopen('K1.raw');
K1 = fread(fid,inf,'double');
fclose(fid);
sz = sqrt( numel(K1) );
K1 = reshape(K1,sz,sz);

figure(1);
clf;
image(K1);

figure(2);
clf;
plot(t,zmp_y,'r');
hold on;
plot(t,com_y,'b');

figure(3);
clf;
plot(t,zmp_x,'r');
hold on;
plot(t,com_x,'b');
