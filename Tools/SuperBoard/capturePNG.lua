local pwd = os.getenv('PWD')
package.cpath = pwd..'/../qt/lib/qt/?.so;'..package.cpath
package.path = pwd..'/../qt/lib/ffi/?.lua;'..package.path

fps = 30;
seconds = 20;
nframes = fps*seconds;

local ffi = require 'ffi'
local libpng = require 'libpng'
local Camera = require "OPCam"
local unix = require 'unix'
Config = {}
Config.camera = require 'Config'
w = Camera.get_width()
h = Camera.get_height()

function camera_init() 
  c = 1;
  for i,auto_param in ipairs(Config.camera.auto_param) do 
    print('Camera : setting '..auto_param.key..': '..auto_param.val[c]); 
    Camera.set_param(auto_param.key, auto_param.val[c]); 
    unix.usleep(100000); 
    print('Camera : set to '..auto_param.key..': '..Camera.get_param(auto_param.key)); 
  end    
  for i,param in ipairs(Config.camera.param) do 
    print('Camera : setting '..param.key..': '..param.val[c]); 
    Camera.set_param(param.key, param.val[c]); 
    unix.usleep(10000); 
    print('Camera : set to '..param.key..': '..Camera.get_param(param.key)); 
  end 
end
--camera_init()

ffi.cdef[[
int poll(struct pullfd *fds, unsigned long nfds, int timeout);
void * malloc( size_t size );
void * free( void* ptr );
]]

-- Initialize Data log
data = {}
--data = ffi.cast('uint32_t**', ffi.C.malloc(nframes) )
for i=1,nframes do
  --data[i] = ffi.cast('uint32_t*', ffi.gc( ffi.C.malloc(w/2 * h), nil ) ) 
  data[i] = ffi.new('uint32_t[?]', w * h / 2 )
end
rgb_data = ffi.new('uint8_t[?]', w * h * 3);

data_t = {};
for i=1,nframes do
  data_t[i] = 0;
end

function save_rgb(rgb)
  saveCount = (saveCount or 0) + 1;
  local filename = string.format("/tmp/rgb_%03d.raw", saveCount);
  local f = io.open(filename, "w+");
  assert(f, "Could not open save image file");
  for i = 1,3*w*h do
    local c = rgb[i];
    if (c < 0) then
      c = 256+c;
    end
    f:write(string.char(c));
  end
  f:close();
end

function yuyv2rgb(yuyv, w, h)
  local countyuv = 0;
  local count = 0;
  for j = 0, h - 1 do
    for i = 0, w - 1 do
      local y1 = bit.rshift(bit.band(yuyv[countyuv], 0x000000FF), 0);
      local u  = bit.rshift(bit.band(yuyv[countyuv], 0x0000FF00), 8);
      local y2 = bit.rshift(bit.band(yuyv[countyuv], 0x00FF0000), 16);
      local v  = bit.rshift(bit.band(yuyv[countyuv], 0xFF000000), 24);
      countyuv = countyuv + 1;
      rgb_data[count] =     2 * (v - 128) + y1 -- R
      rgb_data[count + 1] = 2 * (u - 128) + y1 -- B
      rgb_data[count + 2] = y1 -- G
      rgb_data[count + 3] = 2 * (v - 128) + y2 -- R
      rgb_data[count + 4] = 2 * (u - 128) + y2-- B
      rgb_data[count + 5] = y2 -- G
      count = count + 6
    end
  end
end

-- Capture frames
t0=unix.time()
for i=1,nframes do
  uimage = Camera.get_image();
  while uimage == -1 do
    ffi.C.poll(nil, 0, 1)
    uimage = Camera.get_image()
  end
  --img = ffi.cast('uint32_t*', uimage) 
  ffi.copy(data[i], uimage, w * h * 2 )
  data_t[i] = unix.time()
  --print( 1/(data_t[i]-data_t[i-1]),"FPS" )
end
Camera.stream_off();
Camera.stop();

-- Save Images
for ii=1,nframes do
  print('Saving image ',ii);
  --  save_rgb( data[i] );
  yuyv2rgb( data[ii], w/2, h )
  libpng.save('img/image'..string.format('%04d',ii)..'.png', w, h, rgb_data)
  print('Done!')
end

-- Save timestamp
local f = io.open('img/ts.txt','w');
for ii=1,nframes do
  f:write( tostring(data_t[ii])..'\n' )
end
