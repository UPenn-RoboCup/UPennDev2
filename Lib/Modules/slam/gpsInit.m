function gpsInit
global GPS

id = GetRobotId();

if isempty(GPS) || ~isfield(GPS,'initialized') ||(GPS.initialized ~= 1)
  GPS.msgName = [GetRobotName '/GPS'];
  GPS.utcTime    = [];
  GPS.valid      = 0;
  GPS.lat        = [];
  GPS.NS         = '';
  GPS.lon        = [];
  GPS.EW         = '';
  GPS.speed      = [];
  GPS.heading    = [];
  GPS.date       = [];
  GPS.numPackets = 0;
  GPS.numErrors  = 0;
  GPS.posFix     = 0;
  GPS.numSat     = 0;
  GPS.hdop       = [];
  GPS.mslAlt     = [];
  GPS.geoidSep   = [];
  GPS.magVarDeg  = [];
  GPS.magVarDir  = '';
  GPS.mode       = '';
  GPS.t          = [];
  
 
  
  ipcInit;
  ipcAPIDefine(GPS.msgName,MagicGpsASCIISerializer('getFormat'));
  
  GPS.initialized = 1;
  disp('Gps initialized');
end
  
