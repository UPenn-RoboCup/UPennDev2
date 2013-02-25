module(... or '', package.seeall)

-- Add the required paths
uname  = io.popen('uname -s')
system = uname:read();
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd.."/../Vision/?.lua;"..package.path;

local serialization = require 'serialization'
local util = require 'util'
require 'cutil'

-- Data Type specific
local dataPath = '~/Desktop/logs';
local dataStamp = '02.25.2013.00.*';
local dataTypes = {'lidar','arduimu'}
--local dataTypes = {'lidar'}
local realtime = true;
require 'rcm'
if realtime then
  require 'unix'
end

function get_log_file_list()
  local log_file_list_iter = {};
  for i=1,#dataTypes do
    local log_file = dataTypes[i]..dataStamp..'-'..'*'
    local tmp_file_list = 
    assert(io.popen('/bin/ls '..dataPath..'/'..log_file, 'r'));
    log_file_list_iter[i] = tmp_file_list:lines();
  end
  return log_file_list_iter;
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

-- Parse the data
local parsers_tbl = {}
parsers_tbl[1] = function ( str )
  local lidar_data = serialization.deserialize( str );
  local lidar_tbl = {};
  if lidar_data.arr then
    -- Grab the counter of this lidar scan
    local name = parse_name( lidar_data.arr.name );
    lidar_tbl.counter = name.counter;
    -- Put the ranges into a userdata
    -- TODO: FFI this
    local lidar_ranges = cutil.test_array();
    cutil.string2userdata(lidar_ranges, lidar_data.arr.data);
    lidar_tbl.ranges = lidar_ranges;
    -- Store the timestamp of the data
    lidar_tbl.t = lidar_data.timestamp;
  else
    return nil;
  end
  return lidar_tbl;
end
parsers_tbl[2] = function ( str )
  local imu_data = serialization.deserialize( str );
  print('Parsing IMU!')
  local imu_tbl = {};
  if imu_data.arr then
    -- Grab the counter of this lidar scan
    local name = parse_name( imu_data.arr.name );
    -- Store the timestamp of the data
    imu_tbl.t = imu_data.t;
  else
    return nil;
  end
  return imu_tbl;
end

function push_entry( lidar_tbl )
  rcm.set_lidar_timestamp(lidar_tbl.tstamp);
  rcm.set_lidar_ranges( lidar_tbl.ranges );
  rcm.set_lidar_counter(lidar_tbl.counter);
end

-- Run the main push loop
local log_file_iters = get_log_file_list();

-- Initially open all logs
local entry_iters = {};
-- Save the log handles
local log_handles = {};
-- Save the latest entry
local latest_entry_tbls = {};
for i=1,#log_file_iters do
  local f_tmp = open_log_file( log_file_iters[i]() );
  print('Opening a new file...')
  entry_iters[i] = f_tmp:lines()
  log_handles[i] = f_tmp
  latest_entry_tbls[i] = nil;
end

-- Loop until we say to stop
local last_ts = nil;
local entry_tbls = {};
local entry_timestamps = {}
while true do
  -- Read in datatypes that have not been loaded
  for d=1,#dataTypes do
    -- Only update the blank entries
    if latest_entry_tbls[d]==nil then
      local entry_str = entry_iters[d]();
      if entry_str then
        print('Entry:',entry_str)
        entry_tbls[d] = parsers_tbl[d]( entry_str )
      else
        log_handles[i]:close()
        -- NEED TO OPEN NEW FILE and add an entry!
      end
      -- Store the timestamps, so we can search easily
      entry_timestamps[d] = entry_tbls[d].t;
    end
  end

  -- Who has the min timestamp?
  local min_ts, d_idx = util.min(entry_timestamps)
  print('Pushing',dataTypes[d_idx]);

  -- Push this entry to SHM
  local t_diff = min_ts - (last_ts or min_ts);
  last_ts = min_ts;
  -- If we wish to run in realtime, then sleep sccordingly
  if realtime then
    unix.usleep( 1e6*t_diff );
  end
  push_entry( entry_tbls[d_idx] )

end
