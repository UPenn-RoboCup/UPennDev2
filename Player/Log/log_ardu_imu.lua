-- Include the proper directories
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd..'/../Util/ffi/?.lua;'..package.path

--require 'include'
local ffi = require 'ffi'
local Serial = require('Serial');
local unix = require('unix');
local serialization = require 'serialization'

dev = '/dev/ttyUSB0';
--dev = '/dev/tty.usbserial-A700eEMV'
baud = 115200;
s1 = Serial.connect(dev, baud);

logfile_cnt = 1;
counter = 0;
max_count = 500;
function get_filename()
  local filetime = os.date('%m.%d.%Y.%H.%M');
  local filename = string.format("/mnt/logs/shadwell/logs/arduimu%s-%04d", filetime, logfile_cnt);
  return filename;
end

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
    imu_file = io.open( get_filename() , 'w')
    --imu_file:write( "t Ax Ay Az Wx Wy Wz dt\n" )
  end
  -- Receive data
  imu_data_str, ts = ReceivePacket( 32 );
  -- Write data
  if imu_data_str and #imu_data_str>0 then
    local tbl_data = {}
    vals=string.gmatch(imu_data_str, "%d+")
    tbl_data['Ax'] = tonumber(vals());
    tbl_data['Ay'] = tonumber(vals());
    tbl_data['Az'] = tonumber(vals());
    tbl_data['Wx'] = tonumber(vals());
    tbl_data['Wy'] = tonumber(vals());
    tbl_data['Wz'] = tonumber(vals());
    tbl_data['t'] = ts;
    local s_vals = serialization.serialize( tbl_data );
    imu_file:write( s_vals..'\n' )
    --imu_file:write( ts.." "..imu_data_str.."\n" )
    counter = counter+1;
  end
  
  -- Close file at certain intervals
  if counter>max_count then
		print('Writing',logfile_cnt)
    counter = 0;
    logfile_cnt = logfile_cnt+1;
    imu_file:close()
    imu_file = nil;
  end
end

-- Run the Recording
flush_initial()
while true do
  record()
--  return;
end
