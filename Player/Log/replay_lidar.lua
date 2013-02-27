module(... or '', package.seeall)

-- Add the required paths
uname  = io.popen('uname -s')
system = uname:read();
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd.."/../Vision/?.lua;"..package.path;

local serialization = require 'serialization'
require 'cutil'

-- Data Type specific
local dataPath = '~/Desktop/logs';
local dataStamp = '02.25.2013.00.*';
local dataType = 'lidar'
local realtime = true;
require 'rcm'
if realtime then
  require 'unix'
end

function get_log_file_list()
  -- TODO: use a Lua directory call
  local log_file = dataType..dataStamp..'-'..'*'
  local log_file_list = 
  assert(io.popen('/bin/ls '..dataPath..'/'..log_file, 'r'));
  return log_file_list;
end

function open_log_file( log_file_list )
  local log_f_handle = assert(io.open(log_file_list, 'r+'));
  return log_f_handle;
end

-- Parse the name from a lidar entry
function parse_name(namestr)
  local name = {}
  name.str = namestr:sub(1,namestr:find("%p")-1);
  namestr = namestr:sub( namestr:find('%p')+1 );
  name.counter = tonumber( namestr:sub( 1,namestr:find("%p")-1) );
  namestr = namestr:sub(namestr:find("%p")+1);
  name.partnum = tonumber(namestr:sub(1,namestr:find("%p")-1) )
  namestr = namestr:sub(namestr:find("%p")+1);
  name.parts = tonumber(namestr);
  return name;
end

-- Parse the lidar data
function parse_log_entry( str )
  local lidar_data = serialization.deserialize( str );
  local lidar_tbl = {};
  if lidar_data.arr then
    -- Grab the counter of this lidar scan
    local name = parse_name( lidar_data.arr.name );
    lidar_tbl.counter = name.counter;
    -- Put the ranges into a userdata
    -- TODO: FFI this
    local lidar_ranges = cutil.test_array();
    --print( lidar_data.arr.data )
    cutil.string2userdata(lidar_ranges, lidar_data.arr.data);
    lidar_tbl.ranges = lidar_ranges;
    -- Store the timestamp of the data
    lidar_tbl.tstamp = lidar_data.timestamp;
  else
    return nil;
  end
  return lidar_tbl;
end

function push_entry( lidar_tbl )
  rcm.set_lidar_timestamp(lidar_tbl.tstamp);
  rcm.set_lidar_ranges( lidar_tbl.ranges );
  rcm.set_lidar_counter(lidar_tbl.counter);
end

-- Run the main push loop
local log_file_list = get_log_file_list();
local last_ts = nil;
for l in log_file_list:lines() do
  print('Opening',l)
  local f_handle = open_log_file( l );
  for log_entry in f_handle:lines() do
    local entry_tbl = parse_log_entry( log_entry )
    -- Push to SHM
    local t_diff = entry_tbl.tstamp - (last_ts or entry_tbl.tstamp);
    if realtime then
      unix.usleep( 1e6*t_diff );
    end
    push_entry( entry_tbl )
    last_ts = entry_tbl.tstamp;
  end
  --print('Closing',l)
  f_handle:close()
end

