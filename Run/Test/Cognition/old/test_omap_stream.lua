dofile('../../include.lua')

-- Require the right modules
local simple_ipc = require 'simple_ipc'
require 'unix'
require 'cjpeg'
require 'cutil'

-- Start the timing
local t = unix.time();
local t_last = t;

-- Comm UDP Sending
require 'Comm'
Comm.init('192.168.123.255', 54321);

-- Open logs
log_files = '/tmp/logs_omap/';
f_list = assert(io.popen('/bin/ls '..log_files, 'r'));
f_iter = f_list:lines();

-- *Generic* Timing
local fps = 40;
local inv_fps = 1/40;
for l_file in f_iter do
  t_last = t;
  t = unix.time()
  local t_diff = t-t_last;
  if t_diff<inv_fps then
    unix.usleep( 1e6*(inv_fps-t_diff) )
  end
  local f_log = assert(io.open(log_files..l_file, 'r'))
  local jpeg_log = f_log:read('*all')
  f_log:close()
  local ret = Comm.send( jpeg_log );
  print( 'Sending', l_file, #jpeg_log, ret );
end
