%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GPS message handler 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function slamProcessGps(data,name)

gpsPacket   = MagicGpsASCIISerializer('deserialize',data);
gpsString   = char(gpsPacket.data);
GpsNMEAParser(gpsString);
