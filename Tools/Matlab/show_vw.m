%% open log
% fname = 'plan_1460736484_1.arm';
% fid = fopen(fullfile('/tmp', fname));
% raw = fread(fid,Inf,'*uint8');
% fclose(fid);
% obj = msgpack('unpack', raw);

 %% open other
 %{
load ~/plan0.mat
vwp0 = [vwEffective{:}]';
vw0 = [vwPath{:}]';

load ~/planEnd.mat
vwp = [vwEffective{:}]';
vw = [vwPath{:}]';
%}
%%
load ~/Desktop/pref_vw.mat
figure(1);
subplot(2,2,1);
plot(vwp0);
title('Init vwp');
xlim([1, size(vwp0, 1)]);
rms_pref0 = rms(vwp0);

subplot(2,2,2);
plot(vw0);
title('Init vw');
xlim([1, size(vw0, 1)]);

subplot(2,2,3);
plot(vwp);
title('End vwp');
xlim([1, size(vwp, 1)]);
rms_pref1 = rms(vwp);

subplot(2,2,4);
plot(vw);
xlim([1, size(vw, 1)]);
title('End vw');

%%
load ~/Desktop/rrts_vw.mat
figure(2);
subplot(2,2,1);
plot(vwp0);
title('Init vwp');
xlim([1, size(vwp0, 1)]);

rms_rrts0 = rms(vwp0);


subplot(2,2,2);
plot(vw0);
title('Init vw');
xlim([1, size(vw0, 1)]);

subplot(2,2,3);
plot(vwp);
title('End vwp');
xlim([1, size(vwp, 1)]);

rms_rrts1 = rms(vwp);


subplot(2,2,4);
plot(vw);
xlim([1, size(vw, 1)]);
title('End vw');

%%
fprintf('RMS PREF VWP 0\n');
disp(rms_pref0);
fprintf('RMS PREF VWP 0\n');
disp(rms_pref1);

fprintf('RMS RRTS VWP 0\n');
disp(rms_rrts0);
fprintf('RMS RRTS VWP 1\n');
disp(rms_rrts1);

figure(3);
subplot(2,2,1);
bar(rms_pref0);
title('RMS Pref Original');

subplot(2,2,2);
bar(rms_pref1);
title('RMS Pref Optimized');

subplot(2,2,3);
bar(rms_rrts0);
title('RMS RRT* Original');

subplot(2,2,4);
bar(rms_rrts0);
title('RMS RRT* Optimized');