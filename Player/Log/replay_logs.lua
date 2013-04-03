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
--local mp = require 'ffi/msgpack'
local mp = require 'MessagePack'
require 'Z'
require 'carray'
-- Data Type specific
local dataPath = '~/shadwell/day2_third/';
local dataStamp = '02.27.2013';
local dataTypes = {'flir','lidar','arduimu'}
--local dataTypes = {'flir'}
--local dataTypes = {'lidar','arduimu'}
local dataChannels = {5555,5556,5557}
local realtime = true;
if realtime then
  require 'unix'
end

-- Set up the channels
local simple_ipc = require 'simple_ipc'
ipc_channels = {}
for d=1,#dataTypes do
  ipc_channels[ dataTypes[d] ] = simple_ipc.setup_publisher( dataChannels[d] );
end

function get_log_file_list()
  local log_file_list_iter = {};
  for i=1,#dataTypes do
    local log_file = dataPath..'/logs/'..dataTypes[i]..dataStamp..'*'
    if dataTypes[i]=='flir' then
      log_file = dataPath..'log00_11_1c_01_07_79/fr_*';
    end
    local tmp_file_list = 
    assert(io.popen('/bin/ls '..log_file, 'r'));
    log_file_list_iter[i] = tmp_file_list:lines();
  end
  return log_file_list_iter;
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
parsers_tbl['lidar'] = function ( str )
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
    --lidar_tbl.ranges = Z.compress( lidar_ranges );
    --lidar_tbl.ranges = lidar_data.arr.data;
    local tmp = carray.float(lidar_ranges,1081);
    lidar_tbl.ranges = tmp:string()
    -- Store the timestamp of the data
    lidar_tbl.t = lidar_data.t;
  else
    return nil;
  end
  return lidar_tbl;
end
-- IMU Parser
parsers_tbl['arduimu'] = function ( str )
  local imu_data = serialization.deserialize( str );
--	local imu_tbl = {};
	-- Store the timestamp of the data
--	imu_tbl.t = imu_data.t;
  return imu_data;
end
-- FLIR Parser
parsers_tbl['flir'] = function ( str )
	local flir_tbl = {};
	-- Store the timestamp of the data
  local ts_iter = str[2]:gmatch('%d+[%p%d+]*')
  flir_tbl.counter = tonumber(ts_iter())
  flir_tbl.t = tonumber(ts_iter())
  flir_tbl.data = str[1]
  return flir_tbl;
end

local pushers_tbl = {}
pushers_tbl['lidar'] = function ( lidar_tbl )
  --rcm.set_lidar_ranges( lidar_tbl.ranges );
	--rcm.set_lidar_timestamp(lidar_tbl.t);
	-- Push over ipc (Data and timestamp)
	--ipc_channels['lidar']:send( {lidar_tbl.ranges, lidar_tbl.t} )
	--ipc_channels['lidar']:send( {mp.pack(lidar_tbl.t)} )
	--ipc_channels['lidar']:send( mp.pack({lidar_tbl.t,lidar_tbl.ranges}) )
  local to_send = {};
  to_send.startTime = lidar_tbl.t;
  to_send.ranges = lidar_tbl.ranges;
	ipc_channels['lidar']:send( mp.pack(to_send) )
  --ipc_channels['lidar']:send( lidar_tbl.t..lidar_tbl.ranges );
  --ipc_channels['lidar']:send( lidar_tbl.ranges );
end
pushers_tbl['arduimu'] = function ( imu_tbl )
  local encoded_imu = mp.pack(imu_tbl)
	ipc_channels['arduimu']:send( encoded_imu, #encoded_imu )--send more
end
pushers_tbl['flir'] = function ( flir_tbl )
  if flir_tbl.counter%60==0 then
    local encoded_flir = mp.pack(flir_tbl)
    ipc_channels['flir']:send( encoded_flir, #encoded_flir )--send more
  end
end

function open_log_file( d )
  local log_file_name = log_file_iters[d]()
  if not log_file_name then
    return false
  end
  local log_f_handle = assert(io.open(log_file_name, 'r'));
  if dataTypes[d]=='flir' then
    local ts_file_name = log_file_name:gsub('fr_','ts_')
    local log_ts_handle = assert(io.open(ts_file_name, 'r'));
    -- Update global variables
    log_handles[d] = { log_f_handle, log_ts_handle};
    entry_iters[d] = function()
      local blk_sz = 320*256*2
      local block = log_handles[1][1]:read( blk_sz ) -- Assume flir is first for now
      local ts = log_handles[1][2]:read( '*l' ) -- Assume flir is first for now
      if not ts then return nil end
      if ts:find('unixtime') then
        ts = log_handles[1][2]:read( '*l' )
      end
      return {block,ts};
    end
  else
    log_handles[d] = log_f_handle;
    entry_iters[d] = log_f_handle:lines()
  end
  return true;
end


-- Initialize Global Variables
entry_iters = {};
-- Save the log handles
log_handles = {};
-- Save the latest entry
latest_entry_tbls = {};
-- Set the Log file List
log_file_iters = get_log_file_list();

-- Initial Opening
for d=1,#log_file_iters do
  print('Opening a new '..dataTypes[d]..' file...')
  open_log_file( d );
  latest_entry_tbls[d] = nil;
end

-- Loop until we say to stop
local last_ts = nil;
local entry_timestamps = {}
while true do
  --for trials=1,50 do
  -- Read in datatypes that have not been loaded
  for d=1,#dataTypes do
    -- Only update the blank entries
    if latest_entry_tbls[d]==nil then
      local entry_str = entry_iters[d]();
      if entry_str then
        --print('Entry:', #entry_str, dataTypes[d])
        latest_entry_tbls[d] = parsers_tbl[dataTypes[d]]( entry_str )
      else
        if dataTypes[d]=='flir' then
          log_handles[d][1]:close()
          log_handles[d][2]:close()
        else
          log_handles[d]:close()
        end
        print('Opening a new '..dataTypes[d]..' file...')
        local file_status = open_log_file( d );
        if file_status==false then
          entry_str = nil;
        else
          entry_str = entry_iters[d]();
        end
        if not entry_str then
          print('Done with the '..dataTypes[d]..' logs.')
          latest_entry_tbls[d] = {};
          latest_entry_tbls[d].t = nil;
        else
          latest_entry_tbls[d] = parsers_tbl[ dataTypes[d] ]( entry_str )
        end
      end
      -- Store the timestamps, so we can search easily
      --print(latest_entry_tbls[d].t)
      entry_timestamps[d] = latest_entry_tbls[d].t;
    end
  end

  -- Who has the min timestamp?
  local min_ts, d_idx = util.min(entry_timestamps)

  if not dataTypes[d_idx] then
    print('Done all logs!')
    return
  end

  -- Push this entry to SHM
  local t_diff = min_ts - (last_ts or min_ts);
  last_ts = min_ts;
  
  -- For a particular logfile
  if( min_ts<1361997212.4557 ) then
    realtime = false;
  else
    realtime = true;
  end

  -- If we wish to run in realtime, then sleep accordingly
		-- Only push data when running in realtime
  if realtime then
    unix.usleep( 1e6*t_diff*3 );
		pushers_tbl[ dataTypes[d_idx] ]( latest_entry_tbls[d_idx] )
  end

  -- Empty the data structure
  latest_entry_tbls[d_idx]=nil
end
