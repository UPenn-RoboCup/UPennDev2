
filename = 'msg';
tline = fileread(filename);

msgpack('unpack', tline, size(tline, 2));
