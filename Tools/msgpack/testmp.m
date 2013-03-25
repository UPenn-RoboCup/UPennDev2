clear all;
filename = 'msg';

fid = fopen(filename);
data = fread( fid , '*int8');
whos

fclose(fid)
%tline = fileread(filename);
%for i = 1: numel(tline)
%  fprintf('%u ', tline(i));
%  fprintf('%u ', data(i));
%end

msgpack('unpack', data);
