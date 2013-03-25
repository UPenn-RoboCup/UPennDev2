clear all;
filename = 'msg';

fid = fopen(filename);
data = fread( fid , '*uint8');

fclose(fid);
%tline = fileread(filename);
% for i = 1: numel(data)
% %  fprintf('%u ', tline(i));
%   fprintf('%u ', data(i));
% end

msgpack('unpack', data)
