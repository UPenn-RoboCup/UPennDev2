% Costs
fid = fopen('costs.raw');
data = fread(fid,inf,'*double');
fclose(fid);
%costs = reshape(data,100,100);
costs = reshape(data,388,303);
% Costs to Go
fid = fopen('ctg.raw');
data = fread(fid,inf,'*double');
fclose(fid);
%ctg = reshape(data,100,100);
ctg = reshape(data,388,303);
% Path
fid = fopen('ip1.raw');
%ip1 = fread(fid,inf,'*double');
ip1 = fread(fid,inf,'*int');
disp(numel(ip1));
%ip1 = double(ip1);
fclose(fid);
%
fid = fopen('jp1.raw');
%jp1 = fread(fid,inf,'*double');
jp1 = fread(fid,inf,'*int');
disp(numel(jp1));
%jp1 = double(jp1);
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