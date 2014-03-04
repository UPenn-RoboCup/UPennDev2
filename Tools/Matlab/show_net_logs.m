%% Plot some network data
disp('Loading data...');
clear all;
% NOTE: To convert msgpack data to mat data,
% you must have the msgpack mex file compiled
% It is in the Tools/MATLAB/mex (or similar) directory
% of UPennDev
% After collecting data, please run the following:
%fid = fopen('Data/good_net_load.log');
%msg = fread(fid,'*uint8');
%fclose(fid);
%logs = msgpack('unpacker', msg);
%clear fid msg;
%save('Data/good_net_logs.mat','logs');
%return

load('Data/good_net_logs.mat');

%%
disp('Parsing log headers...');
nlogs = numel(logs) - 3;
% Parse Header data
cmd = char(logs{1});
n = logs{2}.ntimes;
if(n~=nlogs)
    disp('good number of logs');
    return;
end
r = logs{2}.rate;
lut = logs{3};
pids = fieldnames(logs{3});
procs = {};
for i=1:numel(pids)
    procs{i} = char(lut.(pids{i}));
end
m_id = find(ismember(procs,'measure'));
procs(m_id) = [];
clear m_id;
nprocs = numel(procs);

%%
disp('Accumulating logs...');
cpu = zeros(nlogs,nprocs);
mem = zeros(nlogs,nprocs);
ts  = zeros(nlogs,1);
tx  = zeros(nlogs,1);
rx  = zeros(nlogs,1);

for i=1:nlogs
    log = logs{i+3}; %Offset from header
    ts(i) = log.t;
    tx(i) = str2double(char(log.tx));
    rx(i) = str2double(char(log.rx));
    for p=1:nprocs
        load = log.(procs{p});
        cpu(i,p) = str2double(char(load.cpu));
        mem(i,p) = str2double(char(load.mem));
    end
end
%% Post process
disp('Postprocessing data...');
clear logs;
ts = ts - ts(1);
tx = tx - tx(1);
rx = rx - rx(1);

dts = diff(ts);
dtx = diff(tx);
drx = diff(rx);
tss = ts(2:end);
tx_kbps = (dtx./dts)/1024;
rx_kbps = (drx./dts)/1024;

% 5 second windows
windowSize = 5/r;
tx_kbps2 = filter(ones(1,windowSize)/windowSize,1,tx_kbps);
rx_kbps2 = filter(ones(1,windowSize)/windowSize,1,rx_kbps);

%% Plot
disp('Plotting figures...');
f_cpu = figure(1);
set(gca,'FontSize',20);
plot(ts,cpu);
legend(procs,'Location','EastOutside');
xlim([0 n*r]);
xlabel('Time (s)','FontSize',16);
ylabel('CPU (%)','FontSize',16);
title('Computational Utilization','FontSize',24);

f_mem = figure(2);
set(gca,'FontSize',20);
plot(ts,mem);
%legend(procs);
xlim([0 n*r]);
xlabel('Time (s)','FontSize',16);
ylabel('Memory (%)','FontSize',16);
title('Memory Utilization','FontSize',24);

f_net = figure(3);
set(gca,'FontSize',20);
plot(tss,tx_kbps,tss,rx_kbps);
legend('tx','rx','Location','NorthWest');
xlim([0 n*r]);
xlabel('Time (s)','FontSize',16);
ylabel('Data (kBps)','FontSize',16);
title('Network Utilization','FontSize',24);

f_net_ma = figure(4);
set(gca,'FontSize',20);
p_net_ma = plot(tss,tx_kbps2,tss,rx_kbps);
legend('tx','rx','Location','NorthWest');
xlim([0 n*r]);
xlabel('Time (s)','FontSize',16);
ylabel('Data (kBps)','FontSize',16);
title('Network Utilization (moving average)','FontSize',24);

%
%ss = get(0,'ScreenSize');
%set(f_cpu,'units','inches','position',[0 ss(4) 6 3],'Toolbar','None');
%set(f_mem,'units','inches','position',[0 ss(4) 3 3],'Toolbar','None');
%set(f_net,'units','inches','position',[0 ss(4) 3 3],'Toolbar','None');

%% Save figures
disp('Saving figures...');
% Size them
set(f_cpu, 'PaperPositionMode', 'manual','PaperUnits', 'inches');
set(f_mem, 'PaperPositionMode', 'manual','PaperUnits', 'inches');
set(f_net, 'PaperPositionMode', 'manual','PaperUnits', 'inches');
set(f_net_ma, 'PaperPositionMode', 'manual','PaperUnits', 'inches');
set(f_cpu, 'PaperPosition', [0 0 12 6],'PaperSize',[12, 6]);
set(f_mem, 'PaperPosition', [0 0 8 6],'PaperSize',[8, 6]);
set(f_net, 'PaperPosition', [0 0 8 6],'PaperSize',[8, 6]);
set(f_net_ma, 'PaperPosition', [0 0 10 6],'PaperSize',[10, 6]);
%
print(f_cpu,'Figures/load_cpu_good.png','-dpng');
print(f_mem,'Figures/load_mem_good.png','-dpng');
print(f_net,'Figures/load_net_good.png','-dpng');
print(f_net_ma,'Figures/load_net_ma_good.png','-dpng');
%
print(f_cpu,'Figures/load_cpu_good.pdf','-dpdf');
print(f_mem,'Figures/load_mem_good.pdf','-dpdf');
print(f_net,'Figures/load_net_good.pdf','-dpdf');
print(f_net_ma,'Figures/load_net_ma_good.pdf','-dpdf');