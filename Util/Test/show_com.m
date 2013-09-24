fid = fopen('com.tmp');
d = fread(fid,inf,'double');
fclose(fid);

% Massage the data
com = reshape(d,2,numel(d)/2)';

figure(1);
clf;
plot(com);
