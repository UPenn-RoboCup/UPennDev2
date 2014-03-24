dofile('../../include.lua')

local jpeg = require 'jpeg'
w = 320
h = 240
ch = 3;
nbytes = w*h*ch;
if webots then
  ch = 4;--bgra for webots
end
print('Filling a '..w..' by '..h..' image',ch..' channels.')

--if using_luajit then
--  ffi = require 'ffi'
--  img = ffi.new('uint8_t[?]', w*h*ch, 0)
--else
  local carray = require 'carray'
  img = carray.byte(w*h*ch)
--end

for k=1,nbytes,ch do
  -- Blue up top
  img[k] = 0;
  img[k+1] = 0;
  img[k+2] = 255;
  -- Green halfway through
  if k>nbytes/2 then
    img[k] = 255;
    img[k+1] = 255;
    img[k+2] = 0;
  end
end

local c_rgb = jpeg.rgb()
print(c_rgb)
print('compressing')
c_rgb:quality(25)
--os.exit()
--print(jpeg.compress)
--print(c_rgb.compress)
print('ok...')
--img_jpeg = jpeg.compress( c_rgb,img:pointer(), w, h )
img_jpeg = c_rgb:compress( img:pointer(), w, h )




print(type(img_jpeg),'Compression Ratio:', #img_jpeg, #img_jpeg/nbytes )

f = io.open('img.jpeg','w')
n = f:write( img_jpeg )
f:close()
os.exit()


-- gray
ch = 1;
nbytes = w*h*ch;
print('Filling a '..w..' by '..h..' image',ch..' channels.')

--if using_luajit then
--  ffi = require 'ffi'
--  img = ffi.new('uint8_t[?]', w*h*ch, 0)
--else
  local carray = require 'carray'
  img = carray.byte(w*h*ch)
--end

for k=1,nbytes,ch do
  if k>nbytes/2 then
    img[k] = 255;
  else
    img[k] = 0;
  end
end

img_jpeg = jpeg.compress( img:pointer(), w, h, 1 )--gray
print(type(img_jpeg),'Compression Ratio:', #img_jpeg, #img_jpeg/nbytes )

f = io.open('img_gray.jpeg','w')
n = f:write( img_jpeg )
f:close()

ff = io.open('img.jpeg','r')
file_str = ff:read('*a')
print(#file_str)
img_2 = jpeg.uncompress(file_str, #file_str)
print(img_2)
print('width  '..img_2:width())
print('height  '..img_2:height())
print('stride  '..img_2:stride())
--print('color type  '..img_2:color_type())
--print('bit depth  '..img_2:bit_depth())
print('data length  '..img_2:__len())
for i = 1, img_2:stride() * img_2:height(), 3 do
print(img_2[i], img_2[i+1],img_2[i+2])
end

print(img_2:pointer())
