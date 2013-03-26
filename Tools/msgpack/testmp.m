clear all;
%filename = 'msg';
filename = '../../../bus/data/150213185940.20/gpsLocalMP-03.16.2013.15.43.24-0';
%
fid = fopen(filename);
data = fread( fid , '*uint8');

fclose(fid);

%msgpack('unpack', data)

%msg = msgpack('pack', [1,432,43.4]);
%%msg = msgpack('pack', 'fafafaeew23f');
%a = msgpack('unpack', msg)
%%filename = 'msg1';
%%fid = fopen(filename, 'w');
%%data = fwrite(fid, msg, '*uint8');
%%fclose(fid);
%
%msg = msgpack('pack', int16(-32));
%class(msgpack('unpack', msg));
