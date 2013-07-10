-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'

-- Libraries
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local libLaser = require'libLaser'
local messagepack = require 'msgpack'
local udp = require 'udp'
local tutil = require'tutil'
local jpeg = require'jpeg'
local lcm = require'lcm'
local jcm = require'jcm'
local scm = require'scm'
local libHokuyo = require'libHokuyo'
local signal = require'signal'
local vector = require'vector'
require 'unix'

Config = require('ConfigPenn');
use_second_lidar = Config.lidar.use_second_lidar;
lidar_0_head = Config.lidar.lidar_0_head;
local mesh_snd = udp.new_sender(Config.udp.UDP_IP, Config.udp.PORT_LIDAR);
jpeg.set_quality( Config.lidar.depth_quality )
local head_serial = Config.lidar.head_serial
local chest_serial = Config.lidar.chest_serial
local Body = require(Config.Body)

if OPERATING_SYSTEM=='darwin' then
  device0 = "/dev/cu.usbmodem1421"
  device1 = "/dev/cu.usbmodem1422"
end

--Lidar variables
local hokuyo_0 = nil
local hokuyo_1 = nil
local lidar_channel_0 = nil
local lidar_channel_1 = nil
local hokuyo0_poll = {}
local hokuyo1_poll = {}
local wait_channels = {}
lidar_channel_0 = simple_ipc.new_publisher('lidar0') --head lidar
lidar_channel_1 = simple_ipc.new_publisher('lidar1') --chest lidar

-----------------------------------------
--Lidar scan accumulation variables
-----------------------------------------
--Setup lidar data struct

MAX_SCAN_ROWS = 500;
LIDAR_RAY_CENTER = 541;

lidar0 = {};

lidar0.data={};			--Metadata we send to udp
lidar0.data.type =0;		--head lidar
lidar0.data.spineangles={0,0,0,0,0} --waist and neck angles
lidar0.data.lidarangles={}

lidar0.data.range0 =  Config.lidar.lidar0.range0;
lidar0.data.range1 =  Config.lidar.lidar0.range1;
lidar0.data.lidarrange =  Config.lidar.lidar0.lidarrange;

lidar0.data.posex=vector.zeros(MAX_SCAN_ROWS);
lidar0.data.posey=vector.zeros(MAX_SCAN_ROWS);
lidar0.data.posea=vector.zeros(MAX_SCAN_ROWS);



lidar0.count = 0;      		--Counter for lidar scan lines 
lidar0.scan_count=0;   		--Counter for full sweeps
lidar0.range_mag = lidar0.data.range1-lidar0.data.range0 + 1;
lidar0.byte_img = torch.ByteTensor( MAX_SCAN_ROWS, lidar0.range_mag); --depth map

lidar1 = {};

lidar1.data={};			--Metadata we send to udp
lidar1.data.type =1;		--chest lidar
lidar1.data.spineangles={0,0,0,0,0} --waist and neck angles
lidar1.data.lidarangles={}

lidar1.data.range0 =  Config.lidar.lidar1.range0;
lidar1.data.range1 =  Config.lidar.lidar1.range1;
lidar1.data.lidarrange =  Config.lidar.lidar1.lidarrange;

lidar1.data.posex=vector.zeros(MAX_SCAN_ROWS);
lidar1.data.posey=vector.zeros(MAX_SCAN_ROWS);
lidar1.data.posea=vector.zeros(MAX_SCAN_ROWS);


lidar1.count = 0;      		--Counter for lidar scan lines 
lidar1.scan_count=0;   		--Counter for full sweeps
lidar1.range_mag = lidar1.data.range1-lidar1.data.range0 + 1;
lidar1.byte_img = torch.ByteTensor( MAX_SCAN_ROWS, lidar1.range_mag); --depth map

for i=1,MAX_SCAN_ROWS do
  lidar0.data.lidarangles[i]=0;
  lidar1.data.lidarangles[i]=0;
end




function lidar_update(lidartype, data, is_panning)
  local lidar;
  local ranges_str = data[2];

  if lidartype==0 then --head lidar
    lidar = lidar0;
    lidar_scan_count_new = lcm:get_head_lidar_scan_count()[1];
  else
    lidar = lidar1;
    lidar_scan_count_new = lcm:get_chest_lidar_scan_count()[1];
  end

  --A new sweep just started. Flush current image
  if lidar_scan_count_new > lidar.scan_count 
    or lidar.count==MAX_SCAN_ROWS then
    if lidar.count>0 then
      local jdepth = jpeg.compress_gray( 
        lidar.byte_img:storage():pointer(), 
        lidar.range_mag,lidar.count);
      local lidar_str = msgpack.pack({lidar.data,jdepth})
      local udp_ret = mesh_snd:send(lidar_str,#lidar_str)
      print(string.format('Lidar %d Sent Depth Image, size %d', lidartype,udp_ret));
    end
    lidar.count = 0;
    lidar.scan_count = lidar_scan_count_new;
  end
 
  if is_panning==0 then --Not panning
    lidar.count = 0;
    return;
  end
  
  --Accumulate the lidar scaned line into the image
  lidar.count = lidar.count + 1;

  cur_pose = scm:get_pose();
  lidar.data.posex[lidar.count] = cur_pose[1];
  lidar.data.posey[lidar.count] = cur_pose[2];
  lidar.data.posea[lidar.count] = cur_pose[3];

  tutil.byte_cpy_ranges_fov(  lidar.byte_img:select( 1, lidar.count ),
	  ranges_str, 
	  LIDAR_RAY_CENTER+lidar.data.range0, 
	  LIDAR_RAY_CENTER+lidar.data.range1, 
	  lidar.data.lidarrange );

  local jangles= jcm:get_position();
  lidar.data.spineangles=vector.new(
   	{jangles[25],jangles[26],jangles[27],jangles[28],180-jangles[36]}
  	)*math.pi/180;
  if lidartype==0 then --head lidar
     lidar.data.lidarangles[lidar.count] = lidar.data.spineangles[4];
   else
     lidar.data.lidarangles[lidar.count] = lidar.data.spineangles[5];
   end
end


function send_lidar_data(senddata,lidar_num)
  if lidar_num==0 then
    lidar_channel_0:send(senddata);
    lidar0_panning = lcm:get_head_lidar_panning()[1];
    lidar_update(0, senddata,lidar0_panning);
  else
    lidar_channel_1:send(senddata);
    lidar1_panning = lcm:get_chest_lidar_panning()[1];
    lidar_update(1, senddata,lidar1_panning);	
  end
end

--Callback functions
local function hokuyo0_callback()
  local data = hokuyo_0:get_scan()
  local t = unix.time()
  if not data then
	  print('BAD DATA')
	  return
  end
  
  local senddata = { tostring(t), data };
  local metadata = { scm:get_pose_odom() }
  if lidar_0_head then
	  table.insert( metadata, Body.get_neck_position() )
	  table.insert( senddata, messagepack.pack(metadata) )
	  send_lidar_data(senddata,0);
  else
	  table.insert( metadata, Body.get_lidar_position()[1] )
	  table.insert( senddata, messagepack.pack(metadata) )
	  send_lidar_data(senddata,1);
  end 
end

local function hokuyo1_callback()
  local data = hokuyo_1:get_scan()
  local t = unix.time()
  if not data then print('BAD DATA'); return; end
  local senddata = {tostring(t),data};
  local metadata = { scm:get_pose_odom() }
  if lidar_0_head then
	  table.insert( metadata, Body.get_lidar_position()[1] )
	  table.insert( senddata, messagepack.pack(metadata) )
	  send_lidar_data(senddata,1)
  else
	  table.insert( metadata, Body.get_neck_position() )
	  table.insert( senddata, messagepack.pack(metadata) )
	  send_lidar_data(senddata,0)
  end 
end

-- Open the lidars
hokuyo_0 = libHokuyo.open()
print( string.format('Got serial %s',hokuyo_0.info.serial_number))
hokuyo_1 = nil
-- Check if the IDs are swapped
if hokuyo_0.info.serial_number ~= head_serial then
	-- Not the head
	assert(hokuyo_0.info.serial_number==chest_serial,
	"Expected "..chest_serial.." but got "..hokuyo_0.info.serial_number)
	-- Swap
	if use_second_lidar then
		print('Swapped!')
		hokuyo_1 = hokuyo_0
	else
		hokuyo_0:close()
	end
	hokuyo_0 = libHokuyo.open()

	assert(
	hokuyo_0.info.serial_number==head_serial,
	"Expected "..head_serial.." but got "..hokuyo_0.info.serial_number
	)
end
if use_second_lidar and not hokuyo_1 then
	hokuyo_1 = libHokuyo.open()
	print( string.format('Got serial %s',hokuyo_1.info.serial_number))
	if hokuyo_1.info.serial_number ~= chest_serial then
		error('Unknown chest lidar!')
	end
end

-- Setup the callbacks
hokuyo0_poll.socket_handle = hokuyo_0.fd
hokuyo0_poll.callback = hokuyo0_callback;
hokuyo_0:stream_on()
table.insert( wait_channels, hokuyo0_poll )
	

if use_second_lidar then
	if not hokuyo_1 then
		error('Cannot use second lidar!')
	end
  hokuyo1_poll.socket_handle = hokuyo_1.fd
  hokuyo1_poll.callback = hokuyo1_callback;
  hokuyo_1:stream_on()
  table.insert( wait_channels, hokuyo1_poll )
end



-- Ensure that we shutdown the devices properly
function shutdown()
  print'Shutting down the Hokuyos...'
  hokuyo_0:stream_off()
  hokuyo_0:close()
  print'Closed Hokuyo 0'
  if use_second_lidar then
    hokuyo_1:stream_off()
    hokuyo_1:close()
    print'Closed Hokuyo 1'
  end
  error()
end


signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Polling with zeromq
local channel_polls = simple_ipc.wait_on_channels( wait_channels )
local channel_timeout = 100;
while true do
  local npoll = channel_polls:poll(channel_timeout)
end
