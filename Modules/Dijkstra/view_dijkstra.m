% Costs
fid = fopen('costs.raw');
data = fread(fid,inf,'*double');
fclose(fid);
disp( size(data) );
costs = reshape(data,100,100);
% Costs to Go
fid = fopen('ctg.raw');
data = fread(fid,inf,'*double');
fclose(fid);
disp( size(data) );
ctg = reshape(data,100,100);
% Path
fid = fopen('ip1.raw');
ip1 = fread(fid,inf,'*double');
fclose(fid);
%
fid = fopen('jp1.raw');
jp1 = fread(fid,inf,'*double');
fclose(fid);
%
clear data fid
figure(1);
clf;
imagesc(costs);
hold on;
plot(ip1,jp1,'k+');
hold off;
figure(2);
clf;
imagesc(ctg);