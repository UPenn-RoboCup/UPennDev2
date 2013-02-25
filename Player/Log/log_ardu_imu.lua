-- Include the proper directories
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd..'/../Util/ffi/?.lua;'..package.path

--require 'include'
local ffi = require 'ffi'
local Serial = require('Serial');
local unix = require('unix');

--dev = '/dev/ttyUSB0';
dev = '/dev/tty.usbserial-A700eEMV'
baud = 115200;
s1 = Serial.connect(dev, baud);

function ReceivePacket( nbytes ) 
  local buf, buftype, bufsize = Serial.read( nbytes, 5000 );
  local ts = unix.time();
  -- Process the packet
  local str = ffi.string( buf, bufsize );
  -- Apply the straggler
  if( straggle ) then
    str = straggle..str;
    straggle = nil;
  end
  -- Check if a newline is present
  local newline_idx = str:find( '\n' );
  if( newline_idx and newline_idx==nstr ) then
    log_str = str:sub(1,#str-1);
  elseif newline_idx then
    straggle = string.sub( str, newline_idx+1 );
    log_str =  str:sub( 1, newline_idx-1 );
  else
    straggle = str;
  end
  return log_str, ts;
end

function flush_initial()
  local buf, buftype, bufsize = Serial.read( 100, 100 );
  unix.usleep(2e6);
  local buf, buftype, bufsize = Serial.read( 5000, 100 );
  ReceivePacket( 36 ); 
  print( 'Flushed', bufsize );
end

function record()
  -- Make sure we are writing to a file
  if not imu_file then
    imu_file = io.open('imu_'..logfile, 'w')
  end
  -- Receive data
  imu_data_str, ts = ReceivePacket( 36 );
  -- Write data
  if imu_data_str and #imu_data_str>0 then
    print( ts.." "..imu_data_str )
    imu_file:write( ts.." "..imu_data_str.."\n" )
    counter = counter+1;
  end
  
  -- Close file at certain intervals
  if counter%150==0 then
    counter = 0;
    logfile = logfile+1;
    imu_file:close()
    imu_file = nil;
  end
end

-- Run the Recording
flush_initial()
counter = 0;
logfile = 1;
while true do
  record()
--  return;
end
