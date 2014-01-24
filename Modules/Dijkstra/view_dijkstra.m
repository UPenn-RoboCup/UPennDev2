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
%
clear data fid
figure(1);
imagesc(costs);
figure(2);
imagesc(ctg);